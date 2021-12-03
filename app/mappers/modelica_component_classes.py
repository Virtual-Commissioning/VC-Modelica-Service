import math
import fluids

class ModelicaModel:
    
    def __init__(self,package_name = "AutoPackage", model_name = "AutoModel"):
        self.package_name = package_name
        self.model_name = model_name
        self.components = {}
        self.rooms = {}
        self.component_string = ""
        self.room_string = ""
        self.connection_string = '''\n\tequation\n'''
        self.days = 1
    
    def add_component(self,component):
        self.components[component.name] = component
    
    def start_model(self): # Starting text of model - define media
        self.start_string = f''' within {self.package_name};
      model {self.model_name} "Auto-generated model"
        
        package MediumHeating = Buildings.Media.Water(T_default=273.15+70) annotation (
            __Dymola_choicesAllMatching=true);
        package MediumCooling = Buildings.Media.Water(T_default=273.15+5) annotation (
            __Dymola_choicesAllMatching=true);
        package MediumVentilation = Buildings.Media.Air(extraPropertiesNames={{"CO2"}}) annotation (
            __Dymola_choicesAllMatching=true);
        '''
    
    def end_model(self,days): # Last part of model
        stop_time = 24*60*60*days
        self.end_string = f'''
        annotation (experiment(StopTime={stop_time}, __Dymola_Algorithm="Dassl"));
      end {self.model_name};'''

    def create_modelica_package(self):
        self.package_string = f'''within ;
        package {self.package_name}
        annotation (uses(Buildings(version="7.0.1"), Modelica(version="3.2.3"),ToolchainLib));
        end {self.package_name};'''
    
    def create_modelica_model(self):
        self.start_model()
        self.end_model(self.days)
        self.connect_all_components()
        for component in self.components.values():
            component: MS4VCObject
            component.create_component_string()
            self.component_string += component.component_string
        for room in self.rooms.values():
            room: Room
            self.room_string += room.component_string
        self.model_string = self.start_string + self.component_string + self.room_string + self.connection_string + self.end_string
        self.create_modelica_package()

    def map_components(self, system):
        counter = 0 # Counter for distribution of components
        gridwidth = 9 # Width of the visual distribution of the components
        
        ## Special objects:
        self.add_component(Outside(-1, 0, "outside"))
        self.connection_string += self.components["outside"].connect_to_weaBus()
        self.add_component(Plant(-2, 0, "coolingPlant", 0.1, MediumCooling(),5))
        self.add_component(Plant(-3, 0, "heatingPlant", 0.5, MediumHeating(),70))
        
        for component in system:
            x_pos = counter % gridwidth
            y_pos = int((counter - x_pos)/gridwidth)

            if component["ComponentType"] == "FlowSegment":

                obj = Segment(component, x_pos, y_pos)

            elif component["ComponentType"] == "Pump":

                obj = Pump(component, x_pos, y_pos)
                
            elif component["ComponentType"] == "Radiator":

                obj = Radiator(component, x_pos, y_pos)
                
            elif component["ComponentType"] == "HeatExchanger":
                if "ventilation" in component["SystemType"]:
                    continue
                else:
                    obj = HeatingCoil(component, x_pos, y_pos)
                
            elif component["ComponentType"] == "Bend":

                obj = Bend(component, x_pos, y_pos)
                
            elif component["ComponentType"] == "Tee":

                obj = Tee(component, x_pos, y_pos)
                
            elif component["ComponentType"] == "BalancingValve":

                obj = ValveBalancing(component, x_pos, y_pos)
                
            elif component["ComponentType"] == "MotorizedValve":

                obj = ValveMotorized(component, x_pos, y_pos)
                
            elif component["ComponentType"] == "ShuntValve":

                obj = ValveShunt(component, x_pos, y_pos)
                
            elif component["ComponentType"] == "Reduction":

                obj = Reduction(component, x_pos, y_pos)
                
            elif component["ComponentType"] == "AirTerminal":
                
                obj = AirTerminal(component, x_pos, y_pos)

            elif component["ComponentType"] == "MotorizedDamper":
                
                obj = DamperMotorized(component, x_pos, y_pos)

            elif component["ComponentType"] == "BalancingDamper":
                
                obj = DamperBalancing(component, x_pos, y_pos)

            elif component["ComponentType"] == "Fan":
                
                obj = Fan(component, x_pos, y_pos)

            elif component["ComponentType"] == "PressureSensor":

                obj = PressureSensor(component, x_pos, y_pos)
                
                obj.connect_to_outside(self.components["outside"])

            elif component["ComponentType"] == "TemperatureSensor":

                obj = TemperatureSensor(component, x_pos, y_pos)

            else:

                obj = MS4VCObject(component, x_pos, y_pos)

                print(f'''// Component with Tag {component["Tag"]} of type {component["ComponentType"]} not recognized and not mapped.''')
            
            self.add_component(obj)
            
            counter += 1
    
    def connect(self, main_component, connected_component):
        '''
        Function for connecting two components: connected component should always be input to main component
        '''
        # Get name of Modelica input port of current component:
        inport_name = main_component.get_input_port(connected_component)

        # Get name of Modelica output port of connected component:
        outport_name = connected_component.get_output_port(main_component)

        self.connection_string += f'''
        connect({connected_component.modelica_name}.{outport_name},{main_component.modelica_name}.{inport_name}) annotation (Line(points={{{{-46,16}},{{-28,
            16}}}}, color={{0,127,255}}));
            '''

    def connect_all_components(self):
        
        for component in self.components.values():
            
            component: MS4VCObject

            self.connection_string += component.connection_string
            
            if component.FSC_object == None:
                continue
            
            input_tags = [con["Tag"] for con in component.FSC_object["ConnectedWith"] if con["ConnectorType"] == "suppliesFluidFrom" and con["Tag"] != "Not connected"]
            
            if "Not connected" in [con["Tag"] for con in component.FSC_object["ConnectedWith"] if con["ConnectorType"] == "suppliesFluidFrom"]:
                if "ventilation" in component.FSC_object["SystemType"]:
                    self.connect(component, self.components["outside"])
                if "heating" in component.FSC_object["SystemType"]:
                    self.connect(component, self.components["heatingPlant"])
                if "cooling" in component.FSC_object["SystemType"]:
                    self.connect(component, self.components["coolingPlant"])

            elif "Not connected" in [con["Tag"] for con in component.FSC_object["ConnectedWith"] if con["ConnectorType"] == "suppliesFluidTo"]:
                
                if "ventilation" in component.FSC_object["SystemType"]:
                    self.connect(self.components["outside"], component)
                if "heating" in component.FSC_object["SystemType"]:
                    self.connect(self.components["heatingPlant"], component)
                if "cooling" in component.FSC_object["SystemType"]:
                    self.connect(self.components["coolingPlant"], component)

            for input_tag in input_tags:
                self.connect(component, self.components[input_tag])

            if component.control != None:

                sensor_tag = component.control.sensor_tag
                sensor = {**self.components, **self.rooms}[sensor_tag]

                component.control.connect_to_host()
                component.control.connect_to_sensor(sensor)
                
                self.connection_string += component.control.connection_string
    
            if len(component.FSC_object["ContainedInSpaces"]) > 0:
                room_tag = component.FSC_object["ContainedInSpaces"][0]
                self.connection_string += component.connect_to_room(self.rooms[room_tag])
    
    def add_rooms(self, room_list):

        counter = 3 # Counter for distribution of components
        gridwidth = 3 # Width of the visual distribution of the components
        
        for room in room_list:
            
            x_pos = counter % gridwidth
            y_pos = - int((counter - x_pos)/gridwidth)
            new_room = Room(room, x_pos, y_pos)

            self.rooms[new_room.name] = new_room
            self.connection_string += self.rooms[new_room.name].connect_to_weaBus()
            counter += 1

class Medium():
    def __init__(self, name, rho, temp, viscosity):
        self.name = name
        self.rho = rho
        self.temp = temp
        self.viscosity = viscosity

class MediumHeating(Medium):
    def __init__(self):
        super().__init__("MediumHeating", 977, 70, 0.4035e-03)

class MediumCooling(Medium):
    def __init__(self):
        super().__init__("MediumCooling", 1000, 5, 1.5182e-03)

class MediumVentilation(Medium):
    def __init__(self):
        super().__init__("MediumVentilation", 1.2, 20, 1.825e-05)

class MS4VCObject:
    modelica_name_prefix = "c"

    def __init__(self, FSC_object, x_pos, y_pos, name = None):
        self.FSC_object = FSC_object
        if FSC_object != None:
            self.name = FSC_object["Tag"]
            self.modelica_name = self.modelica_name_prefix+self.name
        elif name != None:
            self.name = name
            self.modelica_name = name
        else:
            raise Exception("Either FSC_object or name must be specified!")

        self.find_medium()
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.instantiated_connections = {
            "input": [],
            "output": []
        }
        self.control = None
        self.connection_string = ''
        self.component_string = ''

        self.create_port_names()

    def find_medium(self):
        if "heating" in self.FSC_object["SystemType"]:
            self.medium = MediumHeating()
        elif "ventilation" in self.FSC_object["SystemType"]:
            self.medium = MediumVentilation()
        elif "cooling" in self.FSC_object["SystemType"]:
            self.medium = MediumCooling()
        else:
            raise Exception("Can't determine medium of component")
    
    def create_component_string(self):
        self.component_string += f'''
        // Component with Tag {self.modelica_name} of type {self.FSC_object["ComponentType"]} not recognized.'''

    def create_port_names(self):
        '''
        Default port names for Modelica classes with only two ports
        '''
        self.port_names = {
            "inport": "port_a",
            "outport": "port_b",
            "control": "y"
        }
    
    def get_output_port(self, connected_component):
        if len(self.instantiated_connections["output"]) == 0:
            self.instantiated_connections["output"].append(connected_component.name)
            return self.port_names["outport"]
        else:
            raise Exception("Max output connections for component reached")
    
    def get_input_port(self, connected_component):
        if len(self.instantiated_connections["input"]) == 0:
            self.instantiated_connections["input"].append(connected_component.name)
            return self.port_names["inport"]
        else:
            raise Exception("Max input connections for component reached")

    def get_control_port(self, controller):
        
        return self.port_names["control"]
    
    def calculate_diameters(self):
        """
        Calculates hydraulic diameter of round and rectangular component connections
        """
        hyd_diameters = []
        for conn in [conn for conn in self.FSC_object["ConnectedWith"] if conn != None]:
            if conn["Shape"] == "Round":
                hyd_diameters.append(conn["Dimension"][0])
            elif conn["Shape"] == "Rectangular":
                a = conn["Dimension"][0]
                b = conn["Dimension"][1]
                hyd_diameters.append(2*a*b/(a+b))
        return hyd_diameters
    
    def ispropin(self, prop):
        '''
        Method to check if a property is included in the component. If it is, it will return the property definition.
        '''
        if prop in self.FSC_object.keys():
            return f'{prop} = {self.FSC_object[prop]},'
        else:
            return ''
    
    def calculate_length_between_ports(self, port1,port2):
        len_X = port1["Coordinate"]["X"] - \
            port2["Coordinate"]["X"]
        len_Y = port1["Coordinate"]["Y"] - \
            port2["Coordinate"]["Y"]
        len_Z = port1["Coordinate"]["Z"] - \
            port2["Coordinate"]["Z"]
        length = round(math.sqrt(len_X**2+len_Y**2+len_Z**2),4)
        return length

    def connect_to_room(self, room):
        '''
        Function to combine component with a room. Default is to return an empty string.
        '''
        return ""

class Segment(MS4VCObject):

    modelica_name_prefix = "seg"

    def __init__(self, FSC_object, x_pos, y_pos):

        super().__init__(FSC_object,x_pos, y_pos)

    def create_component_string(self):
        dimension = self.calculate_diameters()[0]
        nom_flow = [conn["DesignFlow"] for conn in self.FSC_object["ConnectedWith"] if conn != None][0]/1000 # m3/s
        m_nom_flow = nom_flow*self.medium.rho
        length = self.calculate_length()

        self.component_string += f'''
        Buildings.Fluid.FixedResistances.Pipe {self.modelica_name}(
            redeclare package Medium = {self.medium.name},
            allowFlowReversal=true,
            m_flow_nominal={round(m_nom_flow,6)},
            thicknessIns=0,
            lambdaIns=0.037,
            diameter={dimension},
            nSeg=2,
            length={length}) 
            annotation (Placement(transformation(extent={{{{{0+self.x_pos*30},{0+self.y_pos*30}}},{{{20+self.x_pos*30},{20+self.y_pos*30}}}}})));
        '''

    def calculate_length(self):
        '''
        Calculates length of a segment
        '''
        if None not in self.FSC_object["ConnectedWith"]:
            length = self.calculate_length_between_ports(self.FSC_object["ConnectedWith"][0],self.FSC_object["ConnectedWith"][1])
        elif "length" in self.FSC_object.keys() and self.FSC_object["length"] == None and None in self.FSC_object["ConnectedWith"]:
            length = None
        return length

class Pump(MS4VCObject):

    modelica_name_prefix = "pump"

    def __init__(self, FSC_object, x_pos, y_pos):

        super().__init__(FSC_object,x_pos, y_pos)

    def create_component_string(self):
        if self.FSC_object["Control"]["ControlType"] == "ConstantSpeedControl":
            self.component_string += f'''
        ToolchainLib.PumpConstantSpeed {self.modelica_name}(
            redeclare package Medium = {self.medium.name},
            speed={self.FSC_object["Control"]["Speed"]},
            pum(
            per(pressure(V_flow={{{', '.join(map(str,list(self.FSC_object["PressureCurve"].keys())))}}}, dp={{{', '.join(map(str,list(self.FSC_object["PressureCurve"].values())))}}}),
            use_powerCharacteristic=true,
            power(V_flow={{{', '.join(map(str,list(self.FSC_object["PowerCurve"].keys())))}}}, P={{{', '.join(map(str,list(self.FSC_object["PowerCurve"].values())))}}}))))
            annotation (Placement(transformation(extent={{{{{0+self.x_pos*30},{0+self.y_pos*30}}},{{{20+self.x_pos*30},{20+self.y_pos*30}}}}})));
            '''
        elif self.FSC_object["Control"]["ControlType"] == "ConstantPressureControl":
            self.component_string += f'''
        ToolchainLib.PumpConstantPressure {self.modelica_name}(
            redeclare package Medium = {self.medium.name},
            pum(p_start(displayUnit="Pa") = {self.FSC_object["Control"]["Pressure"]},
            per(pressure(V_flow={{{', '.join(map(str,list(self.FSC_object["PressureCurve"].keys())))}}}, dp={{{', '.join(map(str,list(self.FSC_object["PressureCurve"].values())))}}}),
            power(V_flow={{{', '.join(map(str,list(self.FSC_object["PowerCurve"].keys())))}}}, P={{{', '.join(map(str,list(self.FSC_object["PowerCurve"].values())))}}}))),
            constPressure(displayUnit="Pa") = {self.FSC_object["Control"]["Pressure"]})
            annotation (Placement(transformation(extent={{{{{0+self.x_pos*30},{0+self.y_pos*30}}},{{{20+self.x_pos*30},{20+self.y_pos*30}}}}})));
            '''
        elif self.FSC_object["Control"]["ControlType"] == "External":
            self.component_string += f'''
        Buildings.Fluid.Movers.SpeedControlled_y {self.modelica_name}(
            redeclare package Medium = {self.medium.name},
            per(pressure(V_flow={{{', '.join(map(str,list(self.FSC_object["PressureCurve"].keys())))}}}, dp={{{', '.join(map(str,list(self.FSC_object["PressureCurve"].values())))}}}),
            use_powerCharacteristic=true,
            power(V_flow={{{', '.join(map(str,list(self.FSC_object["PowerCurve"].keys())))}}}, P={{{', '.join(map(str,list(self.FSC_object["PowerCurve"].values())))}}})))
            annotation (Placement(transformation(extent={{{{{0+self.x_pos*30},{0+self.y_pos*30}}},{{{20+self.x_pos*30},{20+self.y_pos*30}}}}})));
            '''
        else:
            self.component_string += f'''
            // Control type of pump {self.modelica_name} not recognized. 
            '''
    
class Radiator(MS4VCObject):

    modelica_name_prefix = "rad"

    def __init__(self, FSC_object, x_pos, y_pos):

        super().__init__(FSC_object,x_pos, y_pos)

        self.instantiated_connections = {
            "input": [],
            "output": [],
            "heat": []
        }

    def create_component_string(self):
        self.component_string += f'''
            ToolchainLib.Radiator {self.modelica_name}(
                redeclare package Medium = {self.medium.name},
                rad(
                Q_flow_nominal={self.FSC_object["NomPower"]},
                T_a_nominal={self.FSC_object["NomSupplyTemperature"]+273.15},
                T_b_nominal={self.FSC_object["NomReturnTemperature"]+273.15},
                {self.ispropin('n')}
                {self.ispropin('fraRad')}
                dp_nominal={self.FSC_object["NomDp"]},
                TAir_nominal={self.FSC_object["NomRoomTemperature"]+273.15})'''
        if "KV" in self.FSC_object.keys():
            self.component_string += f''',
                trv(Kv={self.FSC_object["KV"]}))
                annotation (Placement(transformation(extent={{{{{0+self.x_pos*30},{0+self.y_pos*30}}},{{{20+self.x_pos*30},{20+self.y_pos*30}}}}})));
                '''
        else:
            self.component_string += f''')
                annotation (Placement(transformation(extent={{{{{0+self.x_pos*30},{0+self.y_pos*30}}},{{{20+self.x_pos*30},{20+self.y_pos*30}}}}})));
            '''

    def create_port_names(self):
        self.port_names = {
            "inport": "port_a",
            "outport": "port_b",
            "heat_port": "heaPor"
        }

    def get_heat_port(self, connected_component):
        self.instantiated_connections["heat"].append(connected_component.name)
        return self.port_names["heat_port"]
    
    def connect_to_room(self, room):
        room_port = room.get_heat_port(self)
        component_port = self.get_heat_port(room)
        color = "191,0,0"

        connection_string = f'''
        connect({self.modelica_name}.{component_port},{room.modelica_name}.{room_port}) annotation (Line(points={{{{-46,16}},{{-28,
            16}}}}, color={{{color}}}));
            '''

        return connection_string

class Bend(MS4VCObject):

    modelica_name_prefix = "bend"

    def __init__(self, FSC_object, x_pos, y_pos):

        super().__init__(FSC_object,x_pos, y_pos)

    def create_component_string(self):
        d = self.calculate_diameters()[0]
        a = (d/2)**2*math.pi
        v_nom_flow = [conn["DesignFlow"] for conn in self.FSC_object["ConnectedWith"] if conn != None][0]/1000 # Flow in m3/s
        m_nom_flow = v_nom_flow*self.medium.rho
        v = v_nom_flow/a
        
        if "Radius" not in self.FSC_object.keys():
            rc = d
        else:
            rc = self.FSC_object["Radius"]

        re = fluids.Reynolds(v,d,rho = self.medium.rho, mu = self.medium.viscosity)
        K = fluids.fittings.bend_rounded(d,90,rc=rc,Re=re)
        dp_nominal = round(K*1/2*self.medium.rho*v**2,4)

        self.component_string += f"""
        Buildings.Fluid.FixedResistances.PressureDrop {self.modelica_name}(
            redeclare package Medium = {self.medium.name},
            m_flow_nominal={round(m_nom_flow,6)},
            dp_nominal={dp_nominal})
            annotation (Placement(transformation(extent={{{{{0+self.x_pos*30},{0+self.y_pos*30}}},{{{20+self.x_pos*30},{20+self.y_pos*30}}}}})));
            """

class HeatingCoil(MS4VCObject):

    modelica_name_prefix = "heatCoil"

    def __init__(self, FSC_object, x_pos, y_pos):

        super().__init__(FSC_object,x_pos, y_pos)

    def create_component_string(self):
        delta_T_A = self.FSC_object["NomSupplyTemperatureSecondary"]-self.FSC_object["NomReturnTemperaturePrimary"]
        delta_T_B = self.FSC_object["NomReturnTemperatureSecondary"]-self.FSC_object["NomSupplyTemperaturePrimary"]
        LMTD = (delta_T_A-delta_T_B)/(math.log(delta_T_A)-math.log(delta_T_B))
        Q = self.FSC_object["NomPower"]
        UA = Q/LMTD

        self.component_string += f'''
        Buildings.Fluid.HeatExchangers.DryCoilCounterFlow {self.modelica_name}(
            redeclare package Medium1 = {MediumVentilation().name},
            redeclare package Medium2 = {self.medium.name},
            m1_flow_nominal={self.FSC_object["NomFlowPrimary"]},
            m2_flow_nominal={self.FSC_object["NomFlowSecondary"]},
            show_T=true,
            dp1_nominal={self.FSC_object["NomDpPrimary"]},
            dp2_nominal={self.FSC_object["NomDpSecondary"]},
            UA_nominal={round(UA,2)})
            annotation (Placement(transformation(extent={{{{{20+self.x_pos*30},{0+self.y_pos*30}}},{{{0+self.x_pos*30},{20+self.y_pos*30}}}}})));
            '''
    
    def create_port_names(self):
        '''
        Port names for heating coils
        '''
        self.port_names = {
            "inport_ventilation": "port_a1",
            "outport_ventilation": "port_b1",
            "inport_water": "port_a2",
            "outport_water": "port_b2"            
        }
    
    def get_output_port(self, connected_component):
        if len(self.instantiated_connections["output"]) >= 2:
            raise Exception("Max output connections (2) for component reached")

        elif "ventilation" in connected_component.FSC_object["SystemType"] or connected_component.FSC_object["ComponentType"] == "HeatExchanger":
            self.instantiated_connections["output"].append(connected_component.name)
            return self.port_names["outport_ventilation"]

        else:
            self.instantiated_connections["output"].append(connected_component.name)
            return self.port_names["outport_water"]
 
    def get_input_port(self, connected_component):
        if len(self.instantiated_connections["input"]) >= 2:
            raise Exception("Max input connections (2) for component reached")

        elif "ventilation" in connected_component.FSC_object["SystemType"] or connected_component.FSC_object["ComponentType"] == "HeatExchanger":
            self.instantiated_connections["input"].append(connected_component.name)
            return self.port_names["inport_ventilation"]
            
        else:
            self.instantiated_connections["input"].append(connected_component.name)
            return self.port_names["inport_water"]
   
class Tee(MS4VCObject):

    modelica_name_prefix = "tee"

    def __init__(self, FSC_object, x_pos, y_pos):

        super().__init__(FSC_object,x_pos, y_pos)

    def create_component_string(self):
        ports = self.FSC_object["ConnectedWith"]
        port_flows = []
        for port in ports:
            if port["ConnectorType"] == "suppliesFluidTo":
                v_nom_flow = -port["DesignFlow"]
                m_nom_flow = round(v_nom_flow*self.medium.rho,6)

                port_flows.append(m_nom_flow)
            
            else:
                v_nom_flow = port["DesignFlow"]
                m_nom_flow = round(v_nom_flow*self.medium.rho,6)

                port_flows.append(m_nom_flow)
            
        self.component_string += f"""
        Buildings.Fluid.FixedResistances.Junction {self.modelica_name}(
            redeclare package Medium = {self.medium.name},
            m_flow_nominal={{{port_flows[0]},{port_flows[1]},{port_flows[2]}}},
            dp_nominal={{0,0,0}})
            annotation (Placement(transformation(extent={{{{{0+self.x_pos*30},{0+self.y_pos*30}}},{{{20+self.x_pos*30},{20+self.y_pos*30}}}}})));
        """

    def create_port_names(self):
        self.port_names = {
            "inport": "port_1",
            "outport": "port_2",
            "secondaryport": "port_3"
        }
    
    def get_output_port(self, connected_component):

        if len(self.instantiated_connections["output"]+self.instantiated_connections["input"]) >= 3:
            raise Exception("Max connections (3) for component reached")
        
        connector_index = [i for i, con in enumerate(self.FSC_object["ConnectedWith"]) if con["Tag"] == connected_component.name]
        
        if len(connector_index) != 1:
            raise Exception(f"Wrong number of matching connectors in component {self.FSC_object['Tag']}")
        else:
            self.instantiated_connections["output"].append(connected_component.name)
            return f"port_{connector_index[0]+1}"
 
    def get_input_port(self, connected_component):
        
        if len(self.instantiated_connections["output"]+self.instantiated_connections["input"]) >= 3:
            raise Exception("Max connections (3) for component reached")
        
        connector_index = [i for i, con in enumerate(self.FSC_object["ConnectedWith"]) if con["Tag"] == connected_component.name]
        
        if len(connector_index) != 1:
            raise Exception(f"Wrong number of matching connectors in component {self.FSC_object['Tag']}")
        else:
            self.instantiated_connections["input"].append(connected_component.name)
            return f"port_{connector_index[0]+1}"

class ValveBalancing(MS4VCObject):

    modelica_name_prefix = "balVal"

    def __init__(self, FSC_object, x_pos, y_pos):

        super().__init__(FSC_object,x_pos, y_pos)
        
        self.connection_string += f"""
        connect({self.modelica_name}.y, {self.modelica_name}_k.y);
        """

    def create_component_string(self):

        nom_flow = [conn["DesignFlow"] for conn in self.FSC_object["ConnectedWith"] if conn != None][0]/1000 # m3/s
        m_nom_flow = nom_flow*self.medium.rho # kg/s

        k = round(self.FSC_object["Kv"]/self.FSC_object["Kvs"],2)

        self.component_string += f"""
        Buildings.Fluid.Actuators.Valves.TwoWayLinear {self.modelica_name}(
            redeclare package Medium = {self.medium.name}, 
            m_flow_nominal= {round(m_nom_flow,6)},
            CvData=Buildings.Fluid.Types.CvTypes.Kv,
            Kv={self.FSC_object["Kvs"]})
            annotation (Placement(transformation(extent={{{{{0+self.x_pos*30},{0+self.y_pos*30}}},{{{20+self.x_pos*30},{20+self.y_pos*30}}}}})));
        
        Modelica.Blocks.Sources.Constant {self.modelica_name}_k(k={k})
            annotation (Placement(transformation(extent={{{{{0+self.x_pos*30},{0+self.y_pos*30}}},{{{20+self.x_pos*30},{20+self.y_pos*30}}}}})));
        """

class ValveMotorized(MS4VCObject):

    modelica_name_prefix = "motVal"

    def __init__(self, FSC_object, x_pos, y_pos):

        super().__init__(FSC_object,x_pos, y_pos)

        self.control = Controller(self,x_pos,y_pos)

        self.component_string += self.control.component_string

    def create_component_string(self):

        nom_flow = [conn["DesignFlow"] for conn in self.FSC_object["ConnectedWith"] if conn != None][0]/1000 # m3/s
        m_nom_flow = nom_flow*self.medium.rho # kg/s

        self.component_string += f"""
        Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage {self.modelica_name}(
            redeclare package Medium = {self.medium.name}, 
            m_flow_nominal= {round(m_nom_flow,6)},
            CvData=Buildings.Fluid.Types.CvTypes.Kv,
            Kv={self.FSC_object["Kvs"]})
            annotation (Placement(transformation(extent={{{{{0+self.x_pos*30},{0+self.y_pos*30}}},{{{20+self.x_pos*30},{20+self.y_pos*30}}}}})));
        """
    
class ValveCheck(MS4VCObject):

    modelica_name_prefix = "cheVal"

    def __init__(self, FSC_object, x_pos, y_pos):

        super().__init__(FSC_object,x_pos, y_pos)

    def create_component_string(self):

        nom_flow = [conn["DesignFlow"] for conn in self.FSC_object["ConnectedWith"] if conn != None][0]/1000 # m3/s
        m_nom_flow = nom_flow*self.medium.rho # kg/s

        self.component_string += f"""
        Buildings.Fluid.FixedResistances.CheckValve {self.modelica_name}(
            m_flow_nominal={round(m_nom_flow,6)},
            CvData=Buildings.Fluid.Types.CvTypes.Kv,
            Kv={self.FSC_object["Kvs"]},
            redeclare package Medium = {self.medium})
            annotation (Placement(transformation(extent={{{{{0+self.x_pos*30},{0+self.y_pos*30}}},{{{20+self.x_pos*30},{20+self.y_pos*30}}}}})));
        """

class ValveShunt(MS4VCObject):

    modelica_name_prefix = "shunt"

    def __init__(self, FSC_object, x_pos, y_pos):

        super().__init__(FSC_object,x_pos, y_pos)

    def create_component_string(self):
        ports = self.FSC_object["ConnectedWith"]
        width = min([self.calculate_length_between_ports(i,j) for j in ports for i in ports if i != j])

        nom_flow = [conn["DesignFlow"] for conn in self.FSC_object["ConnectedWith"] if conn != None][0]/1000 # m3/s
        m_nom_flow = nom_flow*self.medium.rho # kg/s

        self.component_string += f"""
        ToolchainLib.Shunt {self.modelica_name}(res(
            dh={self.FSC_object["ShuntDiameter"]},
            length={width}),
            m_flow_nominal={round(m_nom_flow,6)})
            annotation (Placement(transformation(extent={{{{{0+self.x_pos*30},{0+self.y_pos*30}}},{{{20+self.x_pos*30},{20+self.y_pos*30}}}}})));
        """
    
    def create_port_names(self):
        self.port_names = {
            "inport_supply": "port_a1",
            "outport_supply": "port_b1",
            "inport_return": "port_a2",
            "outport_return": "port_b2"
        }
    
    def get_output_port(self, connected_component):
        if len(self.instantiated_connections["output"]) >= 2:
            raise Exception("Max output connections (2) for component reached")

        elif "retur" in connected_component.FSC_object["SystemName"]:
            self.instantiated_connections["output"].append(connected_component.name)
            return self.port_names["outport_return"]

        else:
            self.instantiated_connections["output"].append(connected_component.name)
            return self.port_names["outport_supply"]
 
    def get_input_port(self, connected_component):
        if len(self.instantiated_connections["input"]) >= 2:
            raise Exception("Max input connections (2) for component reached")

        elif "retur" in connected_component.FSC_object["SystemName"]:
            self.instantiated_connections["input"].append(connected_component.name)
            return self.port_names["inport_return"]
            
        else:
            self.instantiated_connections["input"].append(connected_component.name)
            return self.port_names["inport_supply"]

class Reduction(MS4VCObject):

    modelica_name_prefix = "red"

    def __init__(self, FSC_object, x_pos, y_pos):

        super().__init__(FSC_object,x_pos, y_pos)

    def create_component_string(self):
        
        length = self.calculate_length_between_ports(self.FSC_object["ConnectedWith"][0],self.FSC_object["ConnectedWith"][1])
    
        diameters = self.calculate_diameters()

        directions = [con["ConnectorType"] for con in self.FSC_object["ConnectedWith"]]
        
        inlet_diameter = diameters[directions.index("suppliesFluidFrom")]
        output_diameter = diameters[directions.index("suppliesFluidTo")]

        
        if inlet_diameter>=output_diameter:
            d1 = max(diameters)
            d2 = min(diameters)
            
            k2 = fluids.fittings.contraction_conical_Crane(d1,d2,length)
        
        if inlet_diameter<output_diameter:
            d1 = min(diameters)
            d2 = max(diameters)
            
            k2 = fluids.fittings.diffuser_conical(d1,d2,length,method="Crane")
                
        v_nom_flow = [conn["DesignFlow"] for conn in self.FSC_object["ConnectedWith"] if conn != None][0]/1000 # Flow in m3/s
        m_nom_flow = v_nom_flow*self.medium.rho
        
        v2 = v_nom_flow/((d2/2)**2*math.pi)
        
        dp_nominal = round(k2*1/2*1000*v2**2,4)

        self.component_string += f"""
        Buildings.Fluid.FixedResistances.PressureDrop {self.modelica_name}(
            redeclare package Medium = {self.medium.name},
            m_flow_nominal={round(m_nom_flow,6)},
            dp_nominal={dp_nominal})
            annotation (Placement(transformation(extent={{{{{0+self.x_pos*30},{0+self.y_pos*30}}},{{{20+self.x_pos*30},{20+self.y_pos*30}}}}})));
            """

class Plant(MS4VCObject):
    def __init__(self, x_pos, y_pos, name, nom_flow, medium, temp):
        
        self.nom_flow = nom_flow
        self.temp = temp
        self.medium = medium

        super().__init__(None,x_pos, y_pos, name)

        self.create_port_names()
        

    def find_medium(self):
        '''
        Overwrite find_medium method of super class since it is defined in init method.
        '''
        pass

    def create_component_string(self):
        self.component_string += f"""
        ToolchainLib.GenericPlant {self.modelica_name}(
            redeclare package Medium = {self.medium.name},
            m_flow_nom={self.nom_flow},
            bou(use_T_in=true),
            setpoint={self.temp})
            annotation (Placement(transformation(extent={{{{{0+self.x_pos*30},{0+self.y_pos*30}}},{{{20+self.x_pos*30},{20+self.y_pos*30}}}}})));
        """

class Outside(MS4VCObject):
    def __init__(self, x_pos, y_pos, name = "outside"):

        super().__init__(None, x_pos, y_pos, name)
    
    def find_medium(self):
        '''
        Overwrite find_medium method of super class since it is not used in class.
        '''
        pass

    def create_component_string(self):
        
        nPorts = len(self.instantiated_connections["output"]) + len(self.instantiated_connections["input"])
        
        self.component_string += f"""
        Buildings.Fluid.Sources.Outside {self.modelica_name}(
            redeclare package Medium = {MediumVentilation().name},
            use_C_in=false,
            nPorts={nPorts}) "Outside air conditions"
            annotation (Placement(transformation(extent={{{{{0+self.x_pos*30},{0+self.y_pos*30}}},{{{20+self.x_pos*30},{20+self.y_pos*30}}}}})));
        
        Buildings.BoundaryConditions.WeatherData.Bus weaBus
            annotation (Placement(transformation(extent={{{{{0+self.x_pos*30},{0+(self.y_pos+1)*30}}},{{{20+self.x_pos*30},{20+(self.y_pos+1)*30}}}}})));
        
        Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam=
            Modelica.Utilities.Files.loadResource("modelica://Buildings/Resources/weatherdata/USA_IL_Chicago-OHare.Intl.AP.725300_TMY3.mos"))
            annotation (Placement(transformation(extent={{{{{0+self.x_pos*30},{0+(self.y_pos+2)*30}}},{{{20+self.x_pos*30},{20+(self.y_pos+2)*30}}}}})));
        """

    def create_port_names(self):
        self.port_names = {
            "pressure_port": "ports[3]",
            "inport": "ports[2]",
            "outport": "ports[1]"
        }
    
    def get_output_port(self, connected_component: MS4VCObject):
        port_number = len(self.instantiated_connections["output"]) + len(self.instantiated_connections["input"])+1
        self.instantiated_connections["output"].append(connected_component.name)
        return f"ports[{port_number}]"
    
    def get_input_port(self, connected_component: MS4VCObject):
        port_number = len(self.instantiated_connections["output"]) + len(self.instantiated_connections["input"])+1
        self.instantiated_connections["input"].append(connected_component.name)
        return f"ports[{port_number}]"
    
    def connect_to_weaBus(self):
        connection_string = f'''
        connect({self.modelica_name}.weaBus,weaBus) annotation (Line(points={{{{-46,16}},{{-28,
            16}}}}, color={{255,204,51}}, thickness=0.5));
        
        connect(weaDat.weaBus,weaBus) annotation (Line(points={{{{-46,16}},{{-28,
            16}}}}, color={{255,204,51}}, thickness=0.5));
        '''
        return connection_string

class AirTerminal(MS4VCObject):

    modelica_name_prefix = "term"

    def __init__(self, FSC_object, x_pos, y_pos):

        super().__init__(FSC_object,x_pos, y_pos)

    def create_component_string(self):

        nom_flow = [conn["DesignFlow"] for conn in self.FSC_object["ConnectedWith"] if conn != None][0]/1000 # m3/s
        m_nom_flow = nom_flow*self.medium.rho # kg/s

        dp_nom = (nom_flow*1000/self.FSC_object["Kv"])**2 # Pa

        self.component_string += f"""
        Buildings.Fluid.FixedResistances.PressureDrop {self.modelica_name}(
            redeclare package Medium = {self.medium.name},
            m_flow_nominal={round(m_nom_flow,6)},
            dp_nominal={round(dp_nom, 2)})
            annotation (Placement(transformation(extent={{{{{0+self.x_pos*30},{0+self.y_pos*30}}},{{{20+self.x_pos*30},{20+self.y_pos*30}}}}})));
            """
    
    def connect_to_room(self, room):
        if "fraluft" in self.FSC_object["SystemName"]:
            room_port = room.get_output_port(self)
            component_port = self.get_input_port(room)
        elif "tilluft" in self.FSC_object["SystemName"]:
            room_port = room.get_input_port(self)
            component_port = self.get_output_port(room)
        else:
            raise Exception("Unknown direction of air terminal (supply or return?)")
        
        color = "0,127,255"

        connection_string = f'''
        connect({self.modelica_name}.{component_port},{room.modelica_name}.{room_port}) annotation (Line(points={{{{-46,16}},{{-28,
            16}}}}, color={{{color}}}));
            '''

        return connection_string

class DamperMotorized(MS4VCObject):

    modelica_name_prefix = "motDamp"

    def __init__(self, FSC_object, x_pos, y_pos):

        super().__init__(FSC_object,x_pos, y_pos)

        self.control = Controller(self,x_pos,y_pos)

        self.component_string += self.control.component_string

    def create_component_string(self):

        nom_flow = [conn["DesignFlow"] for conn in self.FSC_object["ConnectedWith"] if conn != None][0]/1000 # m3/s
        m_nom_flow = nom_flow*self.medium.rho # kg/s

        dp_nom = (nom_flow*1000/self.FSC_object["Kvs"])**2 # Pa

        self.component_string += f"""
        Buildings.Fluid.Actuators.Dampers.Exponential {self.modelica_name}(
            redeclare package Medium = {self.medium.name}, 
            m_flow_nominal= {round(m_nom_flow,6)},
            dpDamper_nominal={round(dp_nom, 2)})
            annotation (Placement(transformation(extent={{{{{0+self.x_pos*30},{0+self.y_pos*30}}},{{{20+self.x_pos*30},{20+self.y_pos*30}}}}})));
        """

class DamperBalancing(MS4VCObject):

    modelica_name_prefix = "balDamp"

    def __init__(self, FSC_object, x_pos, y_pos):

        super().__init__(FSC_object,x_pos, y_pos)
        
        self.connection_string += f"""
        connect({self.modelica_name}.y, {self.modelica_name}_k.y);
        """

    def create_component_string(self):

        nom_flow = [conn["DesignFlow"] for conn in self.FSC_object["ConnectedWith"] if conn != None][0]/1000 # m3/s
        m_nom_flow = nom_flow*self.medium.rho # kg/s

        dp_nom = (nom_flow*1000/self.FSC_object["Kvs"])**2 # Pa

        k = round(self.FSC_object["Kv"]/self.FSC_object["Kvs"],2)

        self.component_string += f"""
        Buildings.Fluid.Actuators.Dampers.Exponential {self.modelica_name}(
            redeclare package Medium = {self.medium.name}, 
            m_flow_nominal= {round(m_nom_flow,6)},
            dpDamper_nominal={round(dp_nom, 2)})
            annotation (Placement(transformation(extent={{{{{0+self.x_pos*30},{0+self.y_pos*30}}},{{{20+self.x_pos*30},{20+self.y_pos*30}}}}})));
        
        Modelica.Blocks.Sources.Constant {self.modelica_name}_k(k={k})
            annotation (Placement(transformation(extent={{{{{0+self.x_pos*30},{0+self.y_pos*30}}},{{{20+self.x_pos*30},{20+self.y_pos*30}}}}})));
        """

class Fan(MS4VCObject):
    
    modelica_name_prefix = "fan"

    def __init__(self, FSC_object, x_pos, y_pos):

        super().__init__(FSC_object,x_pos, y_pos)

        self.control = Controller(self,x_pos,y_pos)

        self.component_string += self.control.component_string

    def create_component_string(self):
        self.component_string += f'''
        Buildings.Fluid.Movers.SpeedControlled_y {self.modelica_name}(
            redeclare package Medium = {self.medium.name},
            per(pressure(V_flow={{{', '.join(map(str,list(self.FSC_object["PressureCurve"].keys())))}}}, dp={{{', '.join(map(str,list(self.FSC_object["PressureCurve"].values())))}}}),
            use_powerCharacteristic=true,
            power(V_flow={{{', '.join(map(str,list(self.FSC_object["PowerCurve"].keys())))}}}, P={{{', '.join(map(str,list(self.FSC_object["PowerCurve"].values())))}}})))
            annotation (Placement(transformation(extent={{{{{0+self.x_pos*30},{0+self.y_pos*30}}},{{{20+self.x_pos*30},{20+self.y_pos*30}}}}})));
            '''

class Room(MS4VCObject):
    
    modelica_name_prefix = "room"

    def __init__(self, FSC_object, x_pos, y_pos):
        super().__init__(FSC_object, x_pos, y_pos)

        self.create_component_string()

        self.instantiated_connections = {
            "input": [],
            "output": [],
            "heat": []
        }
    
    def find_medium(self):
        self.medium = MediumVentilation()

    def create_component_string(self):
        self.component_string += f"""
        ToolchainLib.RoomDetached {self.modelica_name}(redeclare package MediumA = 
            {MediumVentilation().name})
            annotation (Placement(transformation(extent={{{{{0+self.x_pos*30},{0+self.y_pos*30}}},{{{20+self.x_pos*30},{20+self.y_pos*30}}}}})));
        """

    def create_port_names(self):
        self.port_names = {
            "inport": "airPorIn",
            "outport": "airPorOut",
            "heat_port": "heaPorAir",
            "weather": "weaBus"
        }

    def get_output_port(self, connected_component):
        self.instantiated_connections["output"].append(connected_component.name)
        return self.port_names["outport"]
        
    def get_input_port(self, connected_component):
        self.instantiated_connections["input"].append(connected_component.name)
        return self.port_names["inport"]

    def get_heat_port(self, connected_component):
        self.instantiated_connections["heat"].append(connected_component.name)
        return self.port_names["heat_port"]

    def connect_to_weaBus(self):
        connection_string = f'''
        connect({self.modelica_name}.weaBus,weaBus) annotation (Line(points={{{{-46,16}},{{-28,
            16}}}}, color={{255,204,51}}, thickness=0.5));
            '''
        return connection_string

    def get_result_port(self, PV_type):
        if PV_type == "Temperature":
            return "airTemp"
        elif PV_type == "CO2":
            return "co2Level"
        else:
            raise Exception(f"Error: {self.__class__.__name__} does not return {PV_type}!")

class PressureSensor(MS4VCObject):

    modelica_name_prefix = "senPre"

    def __init__(self, FSC_object, x_pos, y_pos):

        super().__init__(FSC_object, x_pos, y_pos)

    def create_component_string(self):
        self.component_string += f'''
        ToolchainLib.PressureSensor {self.modelica_name}(redeclare package MediumA = 
            {self.medium.name})
            annotation (Placement(transformation(extent={{{{{0+self.x_pos*30},{0+self.y_pos*30}}},{{{20+self.x_pos*30},{20+self.y_pos*30}}}}})));
            '''

    def create_port_names(self):
        self.port_names = {
            "inport": "port_a",
            "outport": "port_b",
            "room_port": "port_outside",
            "result_port": "statPres"
        }

    def connect_to_outside(self, outside):

        outside_port = outside.get_output_port(self)

        self.connection_string += f'''
        connect({self.modelica_name}.{self.port_names["room_port"]},{outside.name}.{outside_port}) annotation (Line(points={{{{-46,16}},{{-28,
            16}}}}, color={{0,127,255}}));'''
    
    def get_result_port(self, PV_type):
        if PV_type == "Pressure":
            return self.port_names["result_port"]
        else:
            raise Exception(f"Error: {self.__class__.__name__} does not return {PV_type}!")
    
class TemperatureSensor(MS4VCObject):

    modelica_name_prefix = "senTem"

    def __init__(self, FSC_object, x_pos, y_pos):

        super().__init__(FSC_object, x_pos, y_pos)

    def create_component_string(self):
        
        nom_flow = [conn["DesignFlow"] for conn in self.FSC_object["ConnectedWith"] if conn != None][0]/1000 # m3/s
        m_nom_flow = nom_flow*self.medium.rho # kg/s

        self.component_string += f'''
        Buildings.Fluid.Sensors.TemperatureTwoPort {self.modelica_name}(redeclare package Medium = 
            {self.medium.name}, m_flow_nominal={round(m_nom_flow,6)})
            annotation (Placement(transformation(extent={{{{{0+self.x_pos*30},{0+self.y_pos*30}}},{{{20+self.x_pos*30},{20+self.y_pos*30}}}}})));
            '''

    def create_port_names(self):
        self.port_names = {
            "inport": "port_a",
            "outport": "port_b",
            "result_port": "T"
        }
    def get_result_port(self, PV_type):
        if PV_type == "Temperature":
            return self.port_names["result_port"]
        else:
            raise Exception(f"Error: {self.__class__.__name__} does not return {PV_type}!")
 
class Controller:
    def __init__(self, host: MS4VCObject, x_pos, y_pos):
        self.name = "con"+host.name
        self.modelica_name = self.name
        self.host = host
        self.sensor_tag = host.FSC_object["Control"]["ProcessVariableComponentTag"]
        self.control_type = host.FSC_object["Control"]["Type"]
        self.PV_type = host.FSC_object["Control"]["ProcessVariableParameterType"]
        if self.PV_type == "Temperature":
            self.setpoint = host.FSC_object["Control"]["Setpoint"] + 273.15
        else:
            self.setpoint = host.FSC_object["Control"]["Setpoint"]
        self.y_pos = y_pos
        self.x_pos = x_pos
        self.connection_string = ''
        self.component_string = ''
        self.port_names = {
            "sensor_port": "u_m",
            "output_port": "y"
        }
        self.create_component_string()
    
    def connect_to_host(self):
        port_name = self.host.get_control_port(self)
        self.connection_string += f'''
        connect({self.host.modelica_name}.{port_name}, {self.modelica_name}.{self.port_names["output_port"]}) annotation (Line(points={{{{-46,16}},{{-28,
            16}}}}, color={{0,127,255}}));
        '''
        
    def connect_to_sensor(self, sensor: MS4VCObject):
        port_name = sensor.get_result_port(self.PV_type)
        self.connection_string += f'''
        connect({sensor.modelica_name}.{port_name}, {self.modelica_name}.{self.port_names["sensor_port"]}) annotation (Line(points={{{{-46,16}},{{-28,
            16}}}}, color={{0,127,255}}));
        '''
    
    def create_component_string(self):
        if self.host.FSC_object["SystemType"] == "cooling" or self.PV_type == "CO2":
            reverseAction = "true"
        else:
            reverseAction = "false"

        if self.control_type in ["P", "PI", "PD", "PID"]:
            self.component_string += f'''
        ToolchainLib.PIDControl {self.name}(conPID(controllerType=Modelica.Blocks.Types.SimpleController.{self.control_type},
            reverseAction={reverseAction}), setPoint={self.setpoint})
            annotation (Placement(transformation(extent={{{{{0+self.x_pos*30},{0+self.y_pos*30}}},{{{20+self.x_pos*30},{20+self.y_pos*30}}}}})));
        '''

        else:
            raise NotImplementedError("Controllers currently only support PID controllers")
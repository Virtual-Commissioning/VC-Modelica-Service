import math
import fluids

class ModelicaModel:
    
    def __init__(self,package_name = "AutoPackage", model_name = "AutoModel"):
        self.package_name = package_name
        self.model_name = model_name
        self.components = []
        self.component_string = ""
        self.connection_string = '''\n\tequation\n'''
        self.days = 1
    
    def add_component(self,component):
        self.components.append(component)
    
    def start_model(self): # Starting text of model - define media
        self.start_string = f''' within {self.package_name};
      model {self.model_name} "Auto-generated model"
        
        package MediumHeating = Buildings.Media.Water(T_default=273.15+70) annotation (
            __Dymola_choicesAllMatching=true);
        package MediumCooling = Buildings.Media.Water(T_default=273.15+5) annotation (
            __Dymola_choicesAllMatching=true);
        package MediumVentilation = Buildings.Media.Air annotation (
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
        for component in self.components:
            self.component_string += component.component_string
        self.model_string = self.start_string + self.component_string + self.connection_string + self.end_string
        self.create_modelica_package()


    def map_components(self, system):
        counter = 0 # Counter for distribution of components
        gridwidth = 9 # Width of the visual distribution of the components
        for component in system:
            x_pos = counter % gridwidth
            y_pos = int((counter - x_pos)/gridwidth)

            if component["ComponentType"] == "FlowSegment":

                obj = Segment(component, x_pos, y_pos)

                self.add_component(obj)

            elif component["ComponentType"] == "Pump":

                obj = Pump(component, x_pos, y_pos)

                self.add_component(obj)
                
            elif component["ComponentType"] == "Radiator":

                obj = Radiator(component, x_pos, y_pos)

                self.add_component(obj)
                
            elif component["ComponentType"] == "HeatExchanger":

                obj = HeatingCoil(component, x_pos, y_pos)

                self.add_component(obj)
                
            elif component["ComponentType"] == "Bend":

                obj = Bend(component, x_pos, y_pos)

                self.add_component(obj)
                
            elif component["ComponentType"] == "Tee":

                obj = Tee(component, x_pos, y_pos)

                self.add_component(obj)
                
            elif component["ComponentType"] == "BalancingValve":

                obj = ValveBalancing(component, x_pos, y_pos)

                self.add_component(obj)
                
            elif component["ComponentType"] == "MotorizedValve":

                obj = ValveMotorized(component, x_pos, y_pos)

                self.add_component(obj)
                
            elif component["ComponentType"] == "ShuntValve":

                obj = ValveShunt(component, x_pos, y_pos)

                self.add_component(obj)
                
            elif component["ComponentType"] == "Reduction":

                obj = Reduction(component, x_pos, y_pos)

                self.add_component(obj)
                
            else:

                obj = MS4VCObject(component, x_pos, y_pos)

                self.add_component(obj)

                print(f'''// Component with Tag {component["Tag"]} of type {component["ComponentType"]} not recognized and not mapped.''')
            counter += 1

class MS4VCObject:
    model_template = ""
    connector_template = ""
    def __init__(self, FSC_object, x_pos, y_pos):
        self.FSC_object = FSC_object
        self.find_medium()
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.instantiated_connections = {
            "input": [],
            "output": []
        }

        self.create_component_string()

        self.create_port_names()

    def find_medium(self):
        if "heating" in self.FSC_object["SystemType"]:
            self.medium = "MediumHeating"
        elif "ventilation" in self.FSC_object["SystemType"]:
            self.medium = "MediumVentilation"
        elif "cooling" in self.FSC_object["SystemType"]:
            self.medium = "MediumCooling"
        else:
            self.medium = self.FSC_object["SystemType"]
    
    def create_component_string(self):
        self.component_string = f'''
        // Component with Tag {self.FSC_object["Tag"]} of type {self.FSC_object["ComponentType"]} not recognized.'''

    def create_port_names(self):
        '''
        Default port names for Modelica classes with only two ports
        '''
        self.port_names = {
            "inport": "port_a",
            "outport": "port_b"
        }
    
    def get_output_port(self, connected_component):
        if len(self.instantiated_connections["output"]) == 0:
            self.instantiated_connections["output"].append(connected_component["Tag"])
            return self.port_names["outport"]
        else:
            raise Exception("Max output connections for component reached")
    
    def get_input_port(self, connected_component):
        if len(self.instantiated_connections["input"]) == 0:
            self.instantiated_connections["input"].append(connected_component["Tag"])
            return self.port_names["inport"]
        else:
            raise Exception("Max input connections for component reached")
   
    def connect_component(self):
        pass

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
        length = round(math.sqrt(len_X**2+len_Y**2+len_Z**2),2)
        return length

class Segment(MS4VCObject):
    def __init__(self, FSC_object, x_pos, y_pos):

        super().__init__(FSC_object,x_pos, y_pos)

    def create_component_string(self):
        dimension = self.calculate_diameters()[0]
        nom_flow = [conn["DesignFlow"] for conn in self.FSC_object["ConnectedWith"] if conn != None][0]
        length = self.calculate_length()

        self.component_string = f'''
        Buildings.Fluid.FixedResistances.Pipe c{self.FSC_object["Tag"]}(
            redeclare package Medium = {self.medium},
            allowFlowReversal=true,
            m_flow_nominal={nom_flow},
            thicknessIns={None},
            lambdaIns={None},
            diameter={dimension},
            nSeg=2,
            length={length}) 
            annotation (Placement(transformation(extent={{{{{0+self.x_pos*30},{0+self.y_pos*30}}},{{{20+self.x_pos*30},{20+self.y_pos*30}}}}})));
        '''

    def connect_component(self):
        pass # WORK FROM HERE!!
    # WORK FROM HERE!!

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
    def __init__(self, FSC_object, x_pos, y_pos):

        super().__init__(FSC_object,x_pos, y_pos)

    def create_component_string(self):
        if self.FSC_object["Control"]["ControlType"] == "ConstantSpeedControl":
            self.component_string = f'''
        ToolchainLib.PumpConstantSpeed c{self.FSC_object["Tag"]}(
            redeclare package Medium = {self.medium},
            speed={self.FSC_object["Control"]["Speed"]},
            pum(
            per(pressure(V_flow={{{', '.join(map(str,list(self.FSC_object["PressureCurve"].keys())))}}}, dp={{{', '.join(map(str,list(self.FSC_object["PressureCurve"].values())))}}}),
            use_powerCharacteristic=true,
            power(V_flow={{{', '.join(map(str,list(self.FSC_object["PowerCurve"].keys())))}}}, P={{{', '.join(map(str,list(self.FSC_object["PowerCurve"].values())))}}}))))
            annotation (Placement(transformation(extent={{{{{0+self.x_pos*30},{0+self.y_pos*30}}},{{{20+self.x_pos*30},{20+self.y_pos*30}}}}})));
            '''
        elif self.FSC_object["Control"]["ControlType"] == "ConstantPressureControl":
            self.component_string = f'''
        ToolchainLib.PumpConstantPressure c{self.FSC_object["Tag"]}(
            redeclare package Medium = {self.medium},
            pum(p_start(displayUnit="Pa") = {self.FSC_object["Control"]["Pressure"]},
            per(pressure(V_flow={{{', '.join(map(str,list(self.FSC_object["PressureCurve"].keys())))}}}, dp={{{', '.join(map(str,list(self.FSC_object["PressureCurve"].values())))}}}),
            power(V_flow={{{', '.join(map(str,list(self.FSC_object["PowerCurve"].keys())))}}}, P={{{', '.join(map(str,list(self.FSC_object["PowerCurve"].values())))}}}))),
            constPressure(displayUnit="Pa") = {self.FSC_object["Control"]["Pressure"]})
            annotation (Placement(transformation(extent={{{{{0+self.x_pos*30},{0+self.y_pos*30}}},{{{20+self.x_pos*30},{20+self.y_pos*30}}}}})));
            '''
        elif self.FSC_object["Control"]["ControlType"] == "External":
            self.component_string = f'''
        Buildings.Fluid.Movers.SpeedControlled_y c{self.FSC_object["Tag"]}(
            redeclare package Medium = {self.medium},
            per(pressure(V_flow={{{', '.join(map(str,list(self.FSC_object["PressureCurve"].keys())))}}}, dp={{{', '.join(map(str,list(self.FSC_object["PressureCurve"].values())))}}}),
            use_powerCharacteristic=true,
            power(V_flow={{{', '.join(map(str,list(self.FSC_object["PowerCurve"].keys())))}}}, P={{{', '.join(map(str,list(self.FSC_object["PowerCurve"].values())))}}})))
            annotation (Placement(transformation(extent={{{{{0+self.x_pos*30},{0+self.y_pos*30}}},{{{20+self.x_pos*30},{20+self.y_pos*30}}}}})));
            '''
        else:
            self.component_string = f'''
            // Control type of pump {self.FSC_object["Tag"]} not recognized. 
            '''

    def connect_component(self):
        pass # WORK FROM HERE!!
    # WORK FROM HERE!!
    
class Radiator(MS4VCObject):
    def __init__(self, FSC_object, x_pos, y_pos):

        super().__init__(FSC_object,x_pos, y_pos)

    def create_component_string(self):
        self.component_string = f'''
            ToolchainLib.Radiator c{self.FSC_object["Tag"]}(
                redeclare package Medium = {self.medium},
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
    
    def connect_component(self):
        pass # WORK FROM HERE!!
    # WORK FROM HERE!!
    
class Bend(MS4VCObject):
    def __init__(self, FSC_object, x_pos, y_pos):

        super().__init__(FSC_object,x_pos, y_pos)

    def create_component_string(self):
        d = self.calculate_diameters()[0]
        a = (d/2)**2*math.pi
        v_flow = [conn["DesignFlow"] for conn in self.FSC_object["ConnectedWith"] if conn != None][0]/1000 # Flow in m3/s
        m_flow = v_flow*1000
        v = v_flow/a
        
        if "Radius" not in self.FSC_object.keys():
            rc = d
        else:
            rc = self.FSC_object["Radius"]
        
        re = fluids.Reynolds(v,d,nu=4.116e-7)
        K = fluids.fittings.bend_rounded(d,90,rc=rc,Re=re)
        dp_nominal = round(K*1/2*1000*v**2,4)

        self.component_string = f"""
        Buildings.Fluid.FixedResistances.PressureDrop c{self.FSC_object["Tag"]}(
            redeclare package Medium = {self.medium},
            m_flow_nominal={m_flow},
            dp_nominal={dp_nominal})
            annotation (Placement(transformation(extent={{{{{0+self.x_pos*30},{0+self.y_pos*30}}},{{{20+self.x_pos*30},{20+self.y_pos*30}}}}})));
            """
    
    def connect_component(self):
        pass # WORK FROM HERE!!
    # WORK FROM HERE!!

class HeatingCoil(MS4VCObject):
    def __init__(self, FSC_object, x_pos, y_pos):

        super().__init__(FSC_object,x_pos, y_pos)

    def create_component_string(self):
        delta_T_A = self.FSC_object["NomSupplyTemperatureSecondary"]-self.FSC_object["NomReturnTemperaturePrimary"]
        delta_T_B = self.FSC_object["NomReturnTemperatureSecondary"]-self.FSC_object["NomSupplyTemperaturePrimary"]
        LMTD = (delta_T_A-delta_T_B)/(math.log(delta_T_A)-math.log(delta_T_B))
        Q = self.FSC_object["NomPower"]
        UA = Q/LMTD
        self.component_string = f'''
        Buildings.Fluid.HeatExchangers.DryCoilCounterFlow c{self.FSC_object["Tag"]}(
            redeclare package Medium1 = MediumA,
            redeclare package Medium2 = MediumW,
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

        elif "ventilation" in connected_component["FSC_object"]["SystemType"]:
            self.instantiated_connections["output"].append(connected_component["Tag"])
            return self.port_names["outport_ventilation"]

        else:
            self.instantiated_connections["output"].append(connected_component["Tag"])
            return self.port_names["outport_water"]
 
    def get_input_port(self, connected_component):
        if len(self.instantiated_connections["input"]) >= 2:
            raise Exception("Max input connections (2) for component reached")

        elif "ventilation" in connected_component["FSC_object"]["SystemType"]:
            self.instantiated_connections["input"].append(connected_component["Tag"])
            return self.port_names["inport_ventilation"]
            
        else:
            self.instantiated_connections["input"].append(connected_component["Tag"])
            return self.port_names["inport_water"]
   
    def connect_component(self):
        pass # WORK FROM HERE!!
    # WORK FROM HERE!!
    
class Tee(MS4VCObject):

    def __init__(self, FSC_object, x_pos, y_pos):

        super().__init__(FSC_object,x_pos, y_pos)

    def create_component_string(self):
        ports = self.FSC_object["ConnectedWith"]
        port_flows = []
        for port in ports:
            if port["ConnectorType"] == "suppliesFluidTo":
                port_flows.append(-port["DesignFlow"])
            else:
                port_flows.append(port["DesignFlow"])
            
        self.component_string = f"""
        Buildings.Fluid.FixedResistances.Junction c{self.FSC_object["Tag"]}(
            redeclare package Medium = {self.medium},
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

    def connect_component(self):
        pass # WORK FROM HERE!!
    # WORK FROM HERE!!

class ValveBalancing(MS4VCObject):
    def __init__(self, FSC_object, x_pos, y_pos):

        super().__init__(FSC_object,x_pos, y_pos)

    def create_component_string(self):
        self.component_string = f"""
        Buildings.Fluid.Actuators.Valves.TwoWayLinear c{self.FSC_object["Tag"]}(
            redeclare package Medium = {self.medium}, 
            m_flow_nominal= {[conn["DesignFlow"] for conn in self.FSC_object["ConnectedWith"] if conn != None][0]},
            CvData=Buildings.Fluid.Types.CvTypes.Kv,
            Kv={self.FSC_object["Kv"]})
            annotation (Placement(transformation(extent={{{{{0+self.x_pos*30},{0+self.y_pos*30}}},{{{20+self.x_pos*30},{20+self.y_pos*30}}}}})));
        """
    
    def connect_component(self):
        pass # WORK FROM HERE!!
    # WORK FROM HERE!!

class ValveMotorized(MS4VCObject):
    def __init__(self, FSC_object, x_pos, y_pos):

        super().__init__(FSC_object,x_pos, y_pos)

    def create_component_string(self):
        self.component_string = f"""
        Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage c{self.FSC_object["Tag"]}(
            redeclare package Medium = {self.medium}, 
            m_flow_nominal= {[conn["DesignFlow"] for conn in self.FSC_object["ConnectedWith"] if conn != None][0]},
            CvData=Buildings.Fluid.Types.CvTypes.Kv,
            Kv={self.FSC_object["Kvs"]})
            annotation (Placement(transformation(extent={{{{{0+self.x_pos*30},{0+self.y_pos*30}}},{{{20+self.x_pos*30},{20+self.y_pos*30}}}}})));
        """
    
    def connect_component(self):
        pass # WORK FROM HERE!!
    # WORK FROM HERE!!

class ValveCheck(MS4VCObject):
    def __init__(self, FSC_object, x_pos, y_pos):

        super().__init__(FSC_object,x_pos, y_pos)

    def create_component_string(self):
        self.component_string = f"""
        Buildings.Fluid.FixedResistances.CheckValve c{self.FSC_object["Tag"]}(
            m_flow_nominal={[conn["DesignFlow"] for conn in self.FSC_object["ConnectedWith"] if conn != None][0]},
            CvData=Buildings.Fluid.Types.CvTypes.Kv,
            Kv={self.FSC_object["Kvs"]},
            redeclare package Medium = {self.medium})
            annotation (Placement(transformation(extent={{{{{0+self.x_pos*30},{0+self.y_pos*30}}},{{{20+self.x_pos*30},{20+self.y_pos*30}}}}})));
        """
    
    def connect_component(self):
        pass # WORK FROM HERE!!
    # WORK FROM HERE!!

class ValveShunt(MS4VCObject):
    def __init__(self, FSC_object, x_pos, y_pos):

        super().__init__(FSC_object,x_pos, y_pos)

    def create_component_string(self):
        ports = self.FSC_object["ConnectedWith"]
        width = min([self.calculate_length_between_ports(i,j) for j in ports for i in ports if i != j])

        self.component_string = f"""
        ToolchainLib.Shunt c{self.FSC_object["Tag"]}(res(
            m_flow_nominal={[conn["DesignFlow"] for conn in self.FSC_object["ConnectedWith"] if conn != None][0]},
            dh={self.FSC_object["ShuntDiameter"]},
            length={width}))
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

        elif "retur" in connected_component["FSC_object"]["SystemName"]:
            self.instantiated_connections["output"].append(connected_component["Tag"])
            return self.port_names["outport_return"]

        else:
            self.instantiated_connections["output"].append(connected_component["Tag"])
            return self.port_names["outport_supply"]
 
    def get_input_port(self, connected_component):
        if len(self.instantiated_connections["input"]) >= 2:
            raise Exception("Max input connections (2) for component reached")

        elif "retur" in connected_component["FSC_object"]["SystemName"]:
            self.instantiated_connections["input"].append(connected_component["Tag"])
            return self.port_names["inport_return"]
            
        else:
            self.instantiated_connections["input"].append(connected_component["Tag"])
            return self.port_names["inport_supply"]
   
    def connect_component(self):
        pass # WORK FROM HERE!!
    # WORK FROM HERE!!

class Reduction(MS4VCObject):
    def __init__(self, FSC_object, x_pos, y_pos):

        super().__init__(FSC_object,x_pos, y_pos)

    def create_component_string(self):
        length = self.calculate_length_between_ports(self.FSC_object["ConnectedWith"][0],self.FSC_object["ConnectedWith"][1])
    
        diameters = self.calculate_diameters()
        d1 = max(diameters)
        d2 = min(diameters)
        
        k2 = fluids.fittings.contraction_conical_Crane(d1,d2,length)
        
        m_flow = [conn["DesignFlow"] for conn in self.FSC_object["ConnectedWith"] if conn != None][0] # kg/s or l/s
        v_flow = m_flow/1000 # m3/h
        
        v2 = v_flow/((d2/2)**2*math.pi)
        
        dp_nominal = round(k2*1/2*1000*v2**2,4)

        self.component_string = f"""
        Buildings.Fluid.FixedResistances.PressureDrop c{self.FSC_object["Tag"]}(
            redeclare package Medium = {self.medium},
            m_flow_nominal={m_flow},
            dp_nominal={dp_nominal})
            annotation (Placement(transformation(extent={{{{{0+self.x_pos*30},{0+self.y_pos*30}}},{{{20+self.x_pos*30},{20+self.y_pos*30}}}}})));
            """
    
    def connect_component(self):
        pass # WORK FROM HERE!!
    # WORK FROM HERE!!

class Plant(MS4VCObject):
    def __init__(self, x_pos, y_pos, nom_flow, medium):

        super().__init__(None, x_pos, y_pos)
        
        self.nom_flow = nom_flow
        self.medium = medium

    def create_component_string(self):
        self.component_string = f"""
        ToolchainLib.GenericPlant plant(
            redeclare package Medium = {self.medium},
            m_flow_nom={self.nom_flow},
            bou(use_T_in=true),
            setpoint=70)
            annotation (Placement(transformation(extent={{{{{0+self.x_pos*30},{0+self.y_pos*30}}},{{{20+self.x_pos*30},{20+self.y_pos*30}}}}})));
        """
    
    def connect_component(self):
        pass # WORK FROM HERE!!
    # WORK FROM HERE!!
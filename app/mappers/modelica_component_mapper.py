import math

# Defaults for internal parameters - these are changed in the external scripts
package_name = "AutoPackage"
model_name = "AutoModel"
x_pos = 0
y_pos = 0

def calculate_length_between_ports(port1,port2):
    len_X = port1["Coordinate"]["X"] - \
        port2["Coordinate"]["X"]
    len_Y = port1["Coordinate"]["Y"] - \
        port2["Coordinate"]["Y"]
    len_Z = port1["Coordinate"]["Z"] - \
        port2["Coordinate"]["Z"]
    length = round(math.sqrt(len_X**2+len_Y**2+len_Z**2),2)
    return length

def calculate_length(component):
    '''
    Calculates length of a segment
    '''
    length = None
    if None not in component["ConnectedWith"]:
        length = calculate_length_between_ports(component["ConnectedWith"][0],component["ConnectedWith"][1])
    elif "length" in component.keys() and component["length"] == None and None in component["ConnectedWith"]:
        length = None
    return length

def calculate_diameter(comp):
    """
    Calculates hydraulic diameter of round and rectangular pipes and ducts
    """
    if [conn["Shape"] for conn in comp["ConnectedWith"] if conn != None][0] == "Round":
        hyd_diameter = [conn["Dimension"][0] for conn in comp["ConnectedWith"] if conn != None][0]
    elif comp["ConnectedWith"][0]["Shape"] == "Rectangular":
        a = [conn["Dimension"][0] for conn in comp["ConnectedWith"] if conn != None][0]
        b = [conn["Dimension"][1] for conn in comp["ConnectedWith"] if conn != None][0]
        hyd_diameter = 2*a*b/(a+b)
    return hyd_diameter

def calculate_all_diameters(comp):
    hyd_diameters = []
    for conn in [conn for conn in comp["ConnectedWith"] if conn != None]:
        if conn["Shape"] == "Round":
            hyd_diameters.append(conn["Dimension"][0])
        elif conn["Shape"] == "Rectangular":
            a = conn["Dimension"][0]
            b = conn["Dimension"][1]
            hyd_diameters.append(2*a*b/(a+b))
    return hyd_diameters

def model_start(): # Starting text of model - define media
    s = f''' within {package_name};
    model {model_name} "Auto-generated model"
        
        package MediumW = Buildings.Media.Water(T_default=273.15+70) annotation (
            __Dymola_choicesAllMatching=true);
        package MediumA = Buildings.Media.Air annotation (
            __Dymola_choicesAllMatching=true);
    '''
    return s

def model_end(days): # Last part of model
    stop_time = 24*60*60*days
    s = f'''
    annotation (experiment(StopTime={stop_time}, __Dymola_Algorithm="Dassl"));
    end {model_name};'''
    return s

def segment(comp):
    dimension = calculate_diameter(comp)
    nom_flow = [conn["DesignFlow"] for conn in comp["ConnectedWith"] if conn != None][0]
    length = calculate_length(comp)
    s = f'''
        Buildings.Fluid.FixedResistances.Pipe c{comp["Tag"]}(
            redeclare package Medium = MediumW,
            allowFlowReversal=true,
            m_flow_nominal={nom_flow},
            thicknessIns={None},
            lambdaIns={None},
            diameter={dimension},
            nSeg=2,
            length={length}) 
            annotation (Placement(transformation(extent={{{{{0+x_pos*30},{0+y_pos*30}}},{{{20+x_pos*30},{20+y_pos*30}}}}})));
        '''
    return s

def pump(comp):
    if comp["Control"]["ControlType"] == "ConstantSpeedControl":
        s = f'''
        ToolchainLib.PumpConstantSpeed c{comp["Tag"]}(
            redeclare package Medium = MediumW,
            speed={comp["Control"]["Speed"]},
            pum(
            per(pressure(V_flow={{{', '.join(map(str,list(comp["PressureCurve"].keys())))}}}, dp={{{', '.join(map(str,list(comp["PressureCurve"].values())))}}}),
            use_powerCharacteristic=true,
            power(V_flow={{{', '.join(map(str,list(comp["PowerCurve"].keys())))}}}, P={{{', '.join(map(str,list(comp["PowerCurve"].values())))}}}))))
            annotation (Placement(transformation(extent={{{{{0+x_pos*30},{0+y_pos*30}}},{{{20+x_pos*30},{20+y_pos*30}}}}})));
        '''
    elif comp["Control"]["ControlType"] == "ConstantPressureControl":
        s=f'''
        ToolchainLib.PumpConstantPressure c{comp["Tag"]}(
            redeclare package Medium = MediumW,
            pum(p_start(displayUnit="Pa") = {comp["Control"]["Pressure"]},
            per(pressure(V_flow={{{', '.join(map(str,list(comp["PressureCurve"].keys())))}}}, dp={{{', '.join(map(str,list(comp["PressureCurve"].values())))}}}),
            power(V_flow={{{', '.join(map(str,list(comp["PowerCurve"].keys())))}}}, P={{{', '.join(map(str,list(comp["PowerCurve"].values())))}}}))),
            constPressure(displayUnit="Pa") = {comp["Control"]["Pressure"]})
            annotation (Placement(transformation(extent={{{{{0+x_pos*30},{0+y_pos*30}}},{{{20+x_pos*30},{20+y_pos*30}}}}})));
        '''
    elif comp["Control"]["ControlType"] == "External":
        s=f'''
        Buildings.Fluid.Movers.SpeedControlled_y c{comp["Tag"]}(
            redeclare package Medium = MediumW,
            per(pressure(V_flow={{{', '.join(map(str,list(comp["PressureCurve"].keys())))}}}, dp={{{', '.join(map(str,list(comp["PressureCurve"].values())))}}}),
            use_powerCharacteristic=true,
            power(V_flow={{{', '.join(map(str,list(comp["PowerCurve"].keys())))}}}, P={{{', '.join(map(str,list(comp["PowerCurve"].values())))}}})))
            annotation (Placement(transformation(extent={{{{{0+x_pos*30},{0+y_pos*30}}},{{{20+x_pos*30},{20+y_pos*30}}}}})));
        '''
    else:
        s = f'''
        // Control type of pump {comp["Tag"]} not recognized. 
        '''
    return s

def ispropin(prop, comp): # Function to check if a property is included in the component. If it is, it will return the property definition.
    if prop in comp.keys():
        return f'{prop} = {comp[prop]},'
    else:
        return ''

def radiator(comp):
    '''
    Parsing of radiators to custom radiator model including TRV with one heat port and one control port (for setpoint). If room setpoint is specified, it will be written. Otherwise default in model is used.
    '''

    s = f'''
        ToolchainLib.Radiator c{comp["Tag"]}(
            redeclare package Medium = MediumW,
                                        rad(
            Q_flow_nominal={comp["NomPower"]},
            T_a_nominal={comp["NomSupplyTemperature"]+273.15},
            T_b_nominal={comp["NomReturnTemperature"]+273.15},
            {ispropin('n',comp)}
            {ispropin('fraRad',comp)}
            dp_nominal={comp["NomDp"]},
            TAir_nominal={comp["NomRoomTemperature"]+273.15})'''
    if "KV" in comp.keys():
        s+= f''',
            trv(Kv={comp["KV"]}))
            annotation (Placement(transformation(extent={{{{{0+x_pos*30},{0+y_pos*30}}},{{{20+x_pos*30},{20+y_pos*30}}}}})));
            '''
    else:
        s+= f''')
            annotation (Placement(transformation(extent={{{{{0+x_pos*30},{0+y_pos*30}}},{{{20+x_pos*30},{20+y_pos*30}}}}})));
        '''
    return s

def heaCoil(comp):
    
    delta_T_A = comp["NomSupplyTemperatureSecondary"]-comp["NomReturnTemperaturePrimary"]
    delta_T_B = comp["NomReturnTemperatureSecondary"]-comp["NomSupplyTemperaturePrimary"]
    LMTD = (delta_T_A-delta_T_B)/(math.log(delta_T_A)-math.log(delta_T_B))
    Q = comp["NomPower"]
    UA = Q/LMTD
    s = f'''
        Buildings.Fluid.HeatExchangers.DryCoilCounterFlow c{comp["Tag"]}(
            redeclare package Medium1 = MediumA,
            redeclare package Medium2 = MediumW,
            m1_flow_nominal={comp["NomFlowPrimary"]},
            m2_flow_nominal={comp["NomFlowSecondary"]},
            show_T=true,
            dp1_nominal={comp["NomDpPrimary"]},
            dp2_nominal={comp["NomDpSecondary"]},
            UA_nominal={round(UA,2)})
            annotation (Placement(transformation(extent={{{{{20+x_pos*30},{0+y_pos*30}}},{{{0+x_pos*30},{20+y_pos*30}}}}})));
        '''
    return s

def bend(comp):
    import fluids

    d = calculate_diameter(comp)
    a = (d/2)**2*math.pi
    v_flow = [conn["DesignFlow"] for conn in comp["ConnectedWith"] if conn != None][0]/1000 # Flow in m3/s
    m_flow = v_flow*1000
    v = v_flow/a
    
    if "Radius" not in comp.keys():
        rc = d
    else:
        rc = comp["Radius"]
    
    re = fluids.Reynolds(v,d,nu=4.116e-7)
    K = fluids.fittings.bend_rounded(d,90,rc=0.015,Re=re)
    dp_nominal = round(K*1/2*1000*v**2,4)

    s = f"""
        Buildings.Fluid.FixedResistances.PressureDrop c{comp["Tag"]}(
            redeclare package Medium = MediumW,
            m_flow_nominal={m_flow},
            dp_nominal={dp_nominal})
            annotation (Placement(transformation(extent={{{{{0+x_pos*30},{0+y_pos*30}}},{{{20+x_pos*30},{20+y_pos*30}}}}})));
        """
    return s

def tee(comp):
    ports = comp["ConnectedWith"]
    port_flows = []
    for port in ports:
        if port["ConnectorType"] == "suppliesFluidTo":
            port_flows.append(-port["DesignFlow"])
        else:
            port_flows.append(port["DesignFlow"])
        
    s = f"""
        Buildings.Fluid.FixedResistances.Junction c{comp["Tag"]}(
            redeclare package Medium = MediumW,
            m_flow_nominal={{{port_flows[0]},{port_flows[1]},{port_flows[2]}}},
            dp_nominal={{0,0,0}})
            annotation (Placement(transformation(extent={{{{{0+x_pos*30},{0+y_pos*30}}},{{{20+x_pos*30},{20+y_pos*30}}}}})));
        """
    return s

def valve_balancing(comp):
    """
    Mapping for balancing valves
    """

    s = f"""
        Buildings.Fluid.Actuators.Valves.TwoWayLinear c{comp["Tag"]}(
            redeclare package Medium = MediumW, 
            m_flow_nominal= {[conn["DesignFlow"] for conn in comp["ConnectedWith"] if conn != None][0]},
            CvData=Buildings.Fluid.Types.CvTypes.Kv,
            Kv={comp["Kv"]})
            annotation (Placement(transformation(extent={{{{{0+x_pos*30},{0+y_pos*30}}},{{{20+x_pos*30},{20+y_pos*30}}}}})));
    """

    return s


def valve_motorized(comp):
    """
    Mapping for motorized valves
    """

    s = f"""
        Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage c{comp["Tag"]}(
            redeclare package Medium = MediumW, 
            m_flow_nominal= {[conn["DesignFlow"] for conn in comp["ConnectedWith"] if conn != None][0]},
            CvData=Buildings.Fluid.Types.CvTypes.Kv,
            Kv={comp["Kvs"]})
            annotation (Placement(transformation(extent={{{{{0+x_pos*30},{0+y_pos*30}}},{{{20+x_pos*30},{20+y_pos*30}}}}})));
    """

    return s


def valve_check(comp):
    s = f"""
        Buildings.Fluid.FixedResistances.CheckValve c{comp["Tag"]}(
            m_flow_nominal={[conn["DesignFlow"] for conn in comp["ConnectedWith"] if conn != None][0]},
            CvData=Buildings.Fluid.Types.CvTypes.Kv,
            Kv={comp["Kvs"]},
            redeclare package Medium = MediumW)
            annotation (Placement(transformation(extent={{{{{0+x_pos*30},{0+y_pos*30}}},{{{20+x_pos*30},{20+y_pos*30}}}}})));
    """

    return s


def valve_shunt(comp):
    # Find width of shunt:

    ports = comp["ConnectedWith"]
    width = min([calculate_length_between_ports(i,j) for j in ports for i in ports if i != j])

    s = f"""
        ToolchainLib.Shunt c{comp["Tag"]}(res(
            m_flow_nominal={[conn["DesignFlow"] for conn in comp["ConnectedWith"] if conn != None][0]},
            dh={comp["ShuntDiameter"]},
            length={width}))
            annotation (Placement(transformation(extent={{{{{0+x_pos*30},{0+y_pos*30}}},{{{20+x_pos*30},{20+y_pos*30}}}}})));
    """

    return s

def reduction(comp):
    import fluids

    length = calculate_length_between_ports(comp["ConnectedWith"][0],comp["ConnectedWith"][1])
    
    diameters = calculate_all_diameters(comp)
    d1 = max(diameters)
    d2 = min(diameters)
    
    k2 = fluids.fittings.contraction_conical_Crane(d1,d2,length)
    
    m_flow = [conn["DesignFlow"] for conn in comp["ConnectedWith"] if conn != None][0] # kg/s or l/s
    v_flow = m_flow/1000 # m3/h
    
    v2 = v_flow/((d2/2)**2*math.pi)
    
    dp_nominal = round(k2*1/2*1000*v2**2,4)

    s = f"""
        Buildings.Fluid.FixedResistances.PressureDrop c{comp["Tag"]}(
            redeclare package Medium = MediumW,
            m_flow_nominal={m_flow},
            dp_nominal={dp_nominal})
            annotation (Placement(transformation(extent={{{{{0+x_pos*30},{0+y_pos*30}}},{{{20+x_pos*30},{20+y_pos*30}}}}})));
        """
    return s

def plant(system):
    '''
    Instantiates the heating plant as a generic plant with a fixed supply temperature of 70 deg C.
    
    Could be extended with a conditional statement to support definition of the plant type in database.
    '''

    nom_flow = [comp["nom_flow"] for comp in system if "nom_flow" in comp.keys()][0]
    s = f"""
        ToolchainLib.GenericPlant plant(
            redeclare package Medium = MediumW,
            m_flow_nom={nom_flow},
            bou(use_T_in=true),
            setpoint=70)
            annotation (Placement(transformation(extent={{{{{0+x_pos*30},{0+y_pos*30}}},{{{20+x_pos*30},{20+y_pos*30}}}}})));
    """
    return s

def room():
    s = f"""
        ToolchainLib.Room room(redeclare package MediumA = MediumA)
        annotation (Placement(transformation(extent={{{{{0+x_pos*30},{0+y_pos*30}}},{{{20+x_pos*30},{20+y_pos*30}}}}})));
    """
    return s
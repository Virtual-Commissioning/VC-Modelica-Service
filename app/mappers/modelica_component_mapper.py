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
    dimension = [conn["Dimension"] for conn in comp["ConnectedWith"] if conn != None][0]
    s = f'''
        Buildings.Fluid.FixedResistances.Pipe c{comp["Tag"]}(
            redeclare package Medium = MediumW,
            allowFlowReversal=true,
            m_flow_nominal={comp["nom_flow"]},
            thicknessIns={comp["insulation"]["thickness"]},
            lambdaIns={comp["insulation"]["lambda"]},
            diameter={dimension},
            nSeg=2,
            length={comp["length"]}) 
            annotation (Placement(transformation(extent={{{{{0+x_pos*30},{0+y_pos*30}}},{{{20+x_pos*30},{20+y_pos*30}}}}})));
        '''
    return s

def pump(comp):
    if comp["control_type"] == "constant_speed":
        s = f'''
        ToolchainLib.PumpConstantSpeed c{comp["Tag"]}(
            redeclare package Medium = MediumW,
            speed={comp["speed"]},
            pum(per(
            pressure(V_flow={{{', '.join(map(str,comp["per"]["V_flow"]))}}}, dp={{{', '.join(map(str,comp["per"]["pressure"]))}}}),
            use_powerCharacteristic=true,
            power(V_flow={{{', '.join(map(str,comp["per"]["V_flow"]))}}}, P={{{', '.join(map(str,comp["per"]["power"]))}}}))))
            annotation (Placement(transformation(extent={{{{{0+x_pos*30},{0+y_pos*30}}},{{{20+x_pos*30},{20+y_pos*30}}}}})));
        '''
    elif comp["control_type"] == "constant_pressure":
        s=f'''
        ToolchainLib.PumpConstantPressure c{comp["Tag"]}(
            redeclare package Medium = MediumW,
            pum(p_start(displayUnit="Pa") = {comp["Head"]}, per(pressure(V_flow={{{', '.join(map(str,comp["per"]["V_flow"]))}}}, dp={{{', '.join(map(str,comp["per"]["pressure"]))}}}), 
            power(V_flow={{{', '.join(map(str,comp["per"]["V_flow"]))}}}, P={{{', '.join(map(str,comp["per"]["power"]))}}}))),
            constPressure(displayUnit="Pa") = {comp["Head"]})
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
            Q_flow_nominal={comp["nominal_heat_flux"]},
            T_a_nominal={comp["nominal_supply_temp"]+273.15},
            T_b_nominal={comp["nominal_return_temp"]+273.15},
            {ispropin('n',comp)}
            {ispropin('fraRad',comp)}
            dp_nominal={comp["nominal_pressure_loss"]},
            TAir_nominal={comp["nominal_room_temp"]+273.15})'''
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
    import math
    delta_T_A = comp["t_supply_nominal_water"]-comp["t_return_nominal_air"]
    delta_T_B = comp["t_return_nominal_water"]-comp["t_supply_nominal_air"]
    LMTD = (delta_T_A-delta_T_B)/(math.log(delta_T_A)-math.log(delta_T_B))
    Q = comp["Q_flow_nominal"]
    UA = Q/LMTD
    s = f'''
        Buildings.Fluid.HeatExchangers.DryCoilCounterFlow c{comp["Tag"]}(
            redeclare package Medium1 = MediumA,
            redeclare package Medium2 = MediumW,
            m1_flow_nominal={comp["flow_nominal_air"]},
            m2_flow_nominal={comp["flow_nominal_water"]},
            show_T=true,
            dp1_nominal={comp["dp_nominal_air"]},
            dp2_nominal={comp["dp_nominal_water"]},
            UA_nominal={round(UA,2)})
            annotation (Placement(transformation(extent={{{{{20+x_pos*30},{0+y_pos*30}}},{{{0+x_pos*30},{20+y_pos*30}}}}})));
        '''
    return s

def bend(comp):
    import math
    if "Angle" in comp.keys():
        comp["delta"] = round(comp["Angle"]*math.pi/180,4)
    s = f"""
        Modelica.Fluid.Fittings.Bends.CurvedBend c{comp["Tag"]}(redeclare package Medium
            = MediumW, geometry(
            {ispropin('delta',comp)}
            {ispropin('K',comp)}
            d_hyd={comp["Dimension"]},
            R_0={comp["Radius"]}))
            annotation (Placement(transformation(extent={{{{{0+x_pos*30},{0+y_pos*30}}},{{{20+x_pos*30},{20+y_pos*30}}}}})));
        """
    return s

def tee(comp):
    inports = [con["Direction"] for con in comp["ConnectedWith"]].count("In")
    if inports == 1:
        port3_dir = "-1"
    elif inports == 2:
        port3_dir = "1"
    s = f"""
        Buildings.Fluid.FixedResistances.Junction c{comp["Tag"]}(
            redeclare package Medium = MediumW,
            m_flow_nominal={{1,-1,{port3_dir}}},
            dp_nominal={{0,0,0}})
            annotation (Placement(transformation(extent={{{{{0+x_pos*30},{0+y_pos*30}}},{{{20+x_pos*30},{20+y_pos*30}}}}})));
        """
    return s

def valve(comp):
    '''
    Parsing of valves. Kv is currently hardcoded, but could be defined by Kvs and a constant control
    '''
    s = ""
    if "Kv" not in comp.keys():
        comp["Kv"] = comp["Kvs"]
    
    if comp["ValveType"] == "STAD":
        if "Kv" in comp.keys():
            y = comp["Kv"]/comp["Kvs"]
        else:
            y = 1
        s = f"""
        Buildings.Fluid.Actuators.Valves.TwoWayLinear c{comp["Tag"]}(
            redeclare package Medium = MediumW, 
            m_flow_nominal= {comp["nom_flow"]},
            CvData=Buildings.Fluid.Types.CvTypes.Kv,
            Kv={comp["Kvs"]})
            annotation (Placement(transformation(extent={{{{{0+x_pos*30},{0+y_pos*30}}},{{{20+x_pos*30},{20+y_pos*30}}}}})));
        
          Modelica.Blocks.Sources.Constant valCon{comp["Tag"]}(k={y})
          annotation (Placement(transformation(extent={{{{-30,-30}},{{-10,-10}}}})));
        """
    if comp["ValveType"] == "ASV-PV":
        if "Kv" in comp.keys():
            y = comp["Kv"]/comp["Kvs"]
        else:
            y = 1
        s = f"""
        Buildings.Fluid.Actuators.Valves.TwoWayPressureIndependent c{comp["Tag"]}(
            m_flow_nominal={comp["nom_flow"]},
            show_T=true,
            CvData=Buildings.Fluid.Types.CvTypes.Kv,
            Kv={comp["Kvs"]},
            redeclare package Medium = MediumW)
            annotation (Placement(transformation(extent={{{{{0+x_pos*30},{0+y_pos*30}}},{{{20+x_pos*30},{20+y_pos*30}}}}})));
        
          Modelica.Blocks.Sources.Constant valCon{comp["Tag"]}(k={y})
          annotation (Placement(transformation(extent={{{{-30,-30}},{{-10,-10}}}})));
        """
    if comp["ValveType"] == "KONT":
        s = f"""
        Buildings.Fluid.FixedResistances.CheckValve c{comp["Tag"]}(
            m_flow_nominal={comp["nom_flow"]},
            CvData=Buildings.Fluid.Types.CvTypes.Kv,
            Kv={comp["Kvs"]},
            redeclare package Medium = MediumW)
            annotation (Placement(transformation(extent={{{{{0+x_pos*30},{0+y_pos*30}}},{{{20+x_pos*30},{20+y_pos*30}}}}})));
        """
    if comp["ValveType"] == "MOTOR":
        s = f"""
        ToolchainLib.MotorValve c{comp["Tag"]}(
            redeclare package Medium = MediumW,
            tempSP=18,
            Kv={comp["Kvs"]},
            m_flow_nom={comp["nom_flow"]})
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
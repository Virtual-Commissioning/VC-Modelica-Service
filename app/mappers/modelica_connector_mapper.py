mo_file = ""

def connector(comp):
    tags = [con["Tag"] for con in comp["ConnectedWith"] if con != None and "Direction" in con.keys()] # List with tags of connected components
    inout = [con["Direction"] for con in comp["ConnectedWith"] if con != None and "Direction" in con.keys()] # List whether the above components are in- or output
    outportnames = [con["outport_name"] for con in comp["ConnectedWith"] if con != None and "Direction" in con.keys()] # List the Modelica output connector port name for the connectors
    con_string = "" # Instantiate connector string
    
    if "In" in inout and comp["ComponentType"] != "Tee": # All components that have input connectors but tees
        intag = tags[inout.index("In")] # Tag for the input connector
        
        
        if f'''{intag}.{outportnames[inout.index("In")]}''' in mo_file: # If tag of the connected component is already input to another component assume the connected component is a tee and use secondary port (port 3)
            print(f"""{intag} is input to multiple components and is treated as Tee. {comp["Tag"]} has been connected to port_3. \n""") # Create awareness of this
            con_string = f'''
        connect(c{intag}.port_3,c{comp["Tag"]}.{comp["inport_name"]}) annotation (Line(points={{{{-46,16}},{{-28,
            16}},{{-28,0}},{{-10,0}}}}, color={{0,127,255}}));
        '''
        else: # If tag of connected component has not been used before, treat it as normal connection
            con_string = f'''
        connect(c{intag}.{outportnames[inout.index("In")]},c{comp["Tag"]}.{comp["inport_name"]}) annotation (Line(points={{{{-46,16}},{{-28,
            16}},{{-28,0}},{{-10,0}}}}, color={{0,127,255}}));
        '''
        if comp["ComponentType"] == "FlowController" and comp["ValveType"] != "KONT" and comp["ValveType"] != "MOTOR": # If the component is a valve (and not a motor or check valve), also connect it to a constant input
            con_string += f'''
        connect(valCon{comp["Tag"]}.y, c{comp["Tag"]}.y) annotation (Line(points={{{{-46,16}},{{-28,
            16}},{{-28,0}},{{-10,0}}}}, color={{0,127,255}}));
        '''
    
    elif "In" in inout and comp["ComponentType"] == "Tee": # Tees with input connectors
        in_idx = [i for i in range(len(inout)) if inout[i] == "In"] # Index of all input connections
        intags = [tags[i] for i in in_idx]
        outports = [outportnames[i] for i in in_idx]

        # Tees with 1 input
        if len(intags) == 1:
            con_string = f'''
        connect(c{intags[0]}.{outports[0]},c{comp["Tag"]}.{comp["inport_name"]}) annotation (Line(points={{{{-46,16}},{{-28,
            16}},{{-28,0}},{{-10,0}}}}, color={{0,127,255}}));
        '''

        # Tees with 2 inputs
        elif len(intags) ==2:
            con_string = f'''
        connect(c{intags[0]}.{outports[0]},c{comp["Tag"]}.{comp["inport_name"]}) annotation (Line(points={{{{-46,16}},{{-28,
            16}},{{-28,0}},{{-10,0}}}}, color={{0,127,255}}));
        
        connect(c{intags[1]}.{outports[1]},c{comp["Tag"]}.{comp["secondaryport_name"]}) annotation (Line(points={{{{-46,16}},{{-28,
            16}},{{-28,0}},{{-10,0}}}}, color={{0,127,255}}));
        '''

        # Tees with more (or 0) inputs - return error in model
        else:
            con_string = f'''
        // Too many (or 0) input connectors for tee {comp["Tag"]}
            '''
            print(f"""Too many (or 0) input connectors for tee {comp["Tag"]}""")

    elif comp["SystemName"] == "varme, frem": # If there are no inputs and the component is in the heating supply system, treat is as the first component and connect it to the plant
        con_string = f"""
        connect(plant.port_b,c{comp["Tag"]}.{comp["inport_name"]}) annotation (Line(points={{{{-46,16}},{{-28,
            16}},{{-28,0}},{{-10,0}}}}, color={{0,127,255}}));
        """
        print(f"""Could not find input connector for {comp["ComponentType"]} {comp["Tag"]}, which will be treated as start of system. \n""")

    else:
        con_string = f"""
        // Connector for {comp["ComponentType"]} {comp["Tag"]} could not be created.
        """

    
    if "Out" not in inout: # If there is no output connector on the component, treat it as the end of the system and connect it to the plant
        con_string += f"""
        connect(c{comp["Tag"]}.{comp["outport_name"]},plant.port_a) annotation (Line(points={{{{-46,16}},{{-28,
            16}},{{-28,0}},{{-10,0}}}}, color={{0,127,255}}));
        """
        print(f"""Could not find output connector for {comp["ComponentType"]} {comp["Tag"]}, which will be treated as end of system. \n""")

    if comp["ComponentType"] == "Radiator": # Connect radiators to the room
        con_string += f'''
        connect(c{comp["Tag"]}.heatPort, room.heatPort) annotation (Line(points={{{{-46,16}},{{-28,
            16}},{{-28,0}},{{-10,0}}}}, color={{0,127,255}}));
        '''

    if comp["ComponentType"] == "HeatExchanger": # Connect heating coils to the ventilation system
        con_string += f'''
        connect(c{comp["Tag"]}.port_b1, room.port_vent_warm) annotation (Line(points={{{{-46,16}},{{-28,
            16}},{{-28,0}},{{-10,0}}}}, color={{0,127,255}}));

        connect(room.port_vent_cold, c{comp["Tag"]}.port_a1) annotation (Line(points={{{{-46,16}},{{-28,
            16}},{{-28,0}},{{-10,0}}}}, color={{0,127,255}}));
        '''
        
    if comp["ComponentType"] == "FlowController" and comp["ValveType"] == "MOTOR": # Connect motorized valves to the room supply air temperature
        con_string += f'''
        connect(room.inTemp, c{comp["Tag"]}.conInput) annotation (Line(points={{{{-46,16}},{{-28,
            16}},{{-28,0}},{{-10,0}}}}, color={{0,127,255}}));
        '''
    return con_string
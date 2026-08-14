spec = DesignSpec(

        # Trunk
        trunk_length=0.27,          # 2 * 0.135 m
        trunk_width=0.26,           # 2 * 0.13 m
        trunk_thickness=0.19,       # 2 * 0.095 m

        # ABAD actuator
        act_abduction=3,   # TMotor-AK80-9, 24.00 Nm (was "MotorU12_framed+WPG:N=10.50")
        clearance_trunk2abadAct_x=0.0, 
        clearance_trunk2abadAct_y=0.0, 
        clearance_trunk2abadAct_z=0.050,     # "clearance 1" -- abad z-offset

        # Hip Actuator
        act_hip=3,         # TMotor-AK80-9, 24.00 Nm — much lighter/weaker than the original 79.82 Nm
        clearance_abadAct2hipAct_x=0.020,      # hip x-offset clearance
        clearance_abadAct2hipAct_y=0.010,      # hip y-offset clearance (inside the sqrt term)
        clearance_abadAct2hipAct_z=0.0, 

        # Knee Actuator
        act_knee=3,                  # TMotor-AK80-9, 24.00 Nm
        clearance_hipAct2kneeAct_x=0.0, ##constant
        clearance_hipAct2kneeAct_y=0.01, ##constant 
        clearance_hipAct2kneeAct_z=0.0,##constant

        # Thigh Link
        thigh_length=0.2,  # |(-0.200752, -0.286703)| in the sagittal (X-Z) plane
        thigh_width = 0.030. ##constant
        thigh_thickness = 0.015 ##constant
        clearance_kneeAct2thigh_y = 0.0, 

        # Shank Link
        shank_length=0.3,
        shank_width=0.3, ##constant
        shank_thickness=0.3, ##constant

        # Foot Actuator
        act_ankle=4,       # TMotor-AK80-6, 12.00 Nm (was "MotorMN8014_framed+CPG:N=4.00", not in this catalog)
        clearance_shank2footAct_x = 0.0 ##constant
        clearance_shank2footAct_z = 0.0 ##constant

        # Foot link        
        foot_length=0.2,            # 2 * 0.1 m
        foot_width=0.07,            # 2 * 0.035 m
        foot_height=0.03,           # 2 * 0.015 m
        clearance_footAct2foot_x = 0.0 ##constant
        clearance_footAct2foot_z = 0.03 # has to be some positive value ##constant

        # base height from ground
        body_height=0.767,          # base_Link spawn height, matches biped.xml exactly

    )



    fix angle of tghe thigh and shank
def define_rover_1():
    planet = {'g':3.72} #define planet dict with value of g
    power_subsys = {'mass': 90} #define power_subsys dict with value of mass
    science_payload = {'mass': 75}  #define science_payload dict with value of mass
    chassis = {'mass': 659} #define chassis dict with value of mass
    motor = {                    #define motor dict with substructures torque_stall,torque_noload,
             'torque_stall': 170,     #speed_noload,mass
             'torque_noload': 0,
             'speed_noload': 3.8,
             'mass': 5.0
             }
    speed_reducer = {           #define speed_reducer dict with values type,diam_pinion,
                     'type': 'reverted',     #diam_gear,mass
                     'diam_pinion': 0.04,
                     'diam_gear': 0.07,
                     'mass': 1.5}
    wheel = {               #define wheel dict with values radius,mass
             'radius': 0.30,
             'mass': 1.0
             }
    wheel_assembly = {      #define wheel_assembly dict with substructures wheel,speed_reducer
                      'wheel': wheel,     #speed_reducer,motor
                      'speed_reducer': speed_reducer,
                      'motor': motor
                      }
    rover = {                                   #define rover dict with substructures wheel_assembly,
             'wheel_assembly': wheel_assembly,       #chassis,science_payload,power_subsys
             'chassis': chassis,
             'science_payload': science_payload,
             'power_subsys': power_subsys
             }
    return rover, planet

#Imports
from random import randint, seed
from dotenv import load_dotenv
import sys
from tud_sumo.simulation import Simulation
from tud_sumo.plot import Plotter

#Globals
forceGUI=False


#Custom functions

def runScenario(my_sim,useVSL=False):
    # Start the simulation, defining the sumo config files. Add "-gui" to the command line to run with the GUI.
    my_sim.start("a20_exercise/a20_scenario/a20.sumocfg", get_individual_vehicle_data=False, gui=("-gui" in sys.argv) or forceGUI,
                 seed=sim_seed, units="metric") # Units can either be metric (km,kmph)/imperial (mi,mph)/UK (km,mph). All data collected is in these units.
    
    # Add demand from a '.csv' file.
    # my_sim.load_demand("a20_scenario/demand.csv")

    #Add a tracked junction to the intersection with ID "utsc", which will track signal phases/times.
    my_sim.add_tracked_junctions({"utsc": {"flow_params": {"inflow_detectors": ["utsc_n_in_1", "utsc_n_in_2", "utsc_w_in", "utsc_e_in"],
                                                       "outflow_detectors": ["utsc_w_out", "utsc_e_out"],
                                                       "vehicle_types": ["cars", "lorries", "motorcycles", "vans"]}}})

    # Set traffic signal phases. The junc_phases dict can be used for multiple junctions.
    # set_m_phases() can be used to set phases according to different movements (here, movements 1 & 2)
    phases = {"phases": {1: ["G", "y", "r"], 2: ["r", "G", "y"]},
             "times":  {1: [ 27,  3,   20], 2: [ 30,  17,  3]},
             "masks":  {1: "1100",          2: "0011"}}
    
    my_sim.set_m_phases({"utsc": phases})

    # This is equivalent to: my_sim.set_phases({"utsc": {"phases": ["GGrr", "yyrr", "rrGG", "rryy"], "times": [27, 3, 17, 3]}})

    # Add a ramp meter to the junction with ID "crooswijk_meter". The junc_params dict can be used to -> doesn't 'add' anything. the TL is already there and has a certain activation, hardcoded in the network
    # define meter specifc parameters (min/max rate, spillback tracking or queue detectors) and flow specific
    # parameters (inflow/outflow detectors used to calculate in/out flow).
    my_sim.add_tracked_junctions({"crooswijk_meter": {'meter_params': {'min_rate': 200, 'max_rate': 2000, 'queue_detector': "cw_ramp_queue"},
                                                    'flow_params': {'inflow_detectors': ["cw_ramp_inflow", "cw_rm_upstream"], 'outflow_detectors': ["cw_rm_downstream"]}},
                                "a13_meter": {'meter_params': {'min_rate': 200, 'max_rate': 2000, 'queue_detector': "a13_ramp_queue"},
                                                'flow_params': {'inflow_detectors': ["a13_ramp_inflow", "a13_rm_upstream"], 'outflow_detectors': ["a13_rm_downstream"]}}})
    
    # Add Route Guidance (RG) & Variable Speed Limit (VSL) controllers. RG controllers need a detector or
    # edge that will act as the redirection point and a target edge/route ID to redirect drivers to. It is
    # also possible to define a diversion percent to randomly divert a certain percent of drivers, and a
    # highlight colour for the SUMO gui, which will highlight affected drivers. VSL controllers only need
    # to have a list of lanes/edges where they will operate.
    #my_sim.add_controllers({"rerouter": {"type": "RG", "detector_ids": ["rerouter_2"], "new_destination": "urban_out_w", "diversion_pct": 1, "highlight": "00FF00"},
    #                        "vsl": {"type": "VSL", "geometry_ids": ["126729982", "126730069", "126730059"]}})

    my_sim.add_controllers({"vsl1": {"type": "VSL", "geometry_ids": ["629633083", "629633083.833","61121496", "61121498", "54374946", "126730044", "126729958", "126710337"]}})

    my_sim.add_controllers({"vsl2": {"type": "VSL", "geometry_ids": ["1191885780", "1191885781","1191885778", "126730088", "491000664", "29324581", "1191885774"]}})


    # Add tracked edges. This will track some basic information, such as average speed etc, but can also
    # be used to create space-time diagrams as individual vehicle speeds and positions are tracked.
    tracked_edges = ["629633083", "629633083.833", "61121496", "61121498", "54374946" , "126730044" , "126729958" , "126710337" , "1191885785", "699077562", "699077563",  "487223604",  "1191885783",  "1191885780",  "1191885781" , "1191885778", "126730088", "491000664" , "29324581",  "1191885774" , "126730026"]
    my_sim.add_tracked_edges(tracked_edges)

    # Add scheduled events from a JSON file (can be dictionary). Use the format as in example_incident.json
    #my_sim.add_events("a20_exercise/a20_scenario/example_incident.json")

    # Add a new route that vehicles can be assigned to.
    my_sim.add_route(("urban_in_e", "urban_out_w"), "new_route")

    # These individual functions above can be replaced as below, where the 'parameters.json' file contains
    # a dictionary of all necessary parameters (under 'edges', 'junctions', 'phases', 'controllers' and 'events')
    # my_sim.load_objects("parameters.json")

    # This file can either be created manually, or by saving objects in previous simulations. This is done
    # using the save_objects function as below.
    my_sim.save_objects("objects.json")

    # Add a function that is called on each new vehicle in the simulation. Simulation parameters are; curr_step,
    # vehicle_id, route_id, vehicle_type, departure, origin, destination. These values are filled automatically.
    # For other parameters, use a parameters dictionary as below. Use add_vehicle_out_funcs() for functions
    # called when vehicles exit the simulation (only with vehicle_id and/or curr_step). Vehicle in/out functions
    # can be removed using remove_vehicle_[in/out]_functions().

    # MR: wonder if I can use this to flexibly change the vehicle type on the fly?

    # vehicle_ids, new_veh_idx = [], 0
    # def add_to_vehicle_arr(simulation, vehicle_id, arr):
    #     """
    #     Append vehicle ID and initial speed to an array.
    #     """
    #     veh_speed = simulation.get_vehicle_vals(vehicle_id, "speed")
    #     arr.append((vehicle_id, veh_speed))
    
    def stochastic_av_vehicle_class(simulation, vehicle_id, avShare=0):
        """
        Reassign vehicle ID to AV type if random roll < avShare
        """
        if randint(0,100)/100 < avShare:
            if simulation.vehicle_exists(vehicle_id):
                simulation.set_vehicle_vals(vehicle_id,type="AV")

    my_sim.add_vehicle_in_functions(stochastic_av_vehicle_class, parameters={"avShare": avShare})
    # my_sim.add_vehicle_in_functions(add_to_vehicle_arr, parameters={"arr": vehicle_ids})

    n, sim_dur, warmup = 1, 2000 / my_sim.step_length, 0 / my_sim.step_length
    
    my_sim.set_tl_metering_rate(rm_id="crooswijk_meter", metering_rate=2000) #2000 should boil down to 'allow all'
    my_sim.set_tl_metering_rate(rm_id="a13_meter", metering_rate=2000)
    
    if warmup > 0:
        my_sim.step_through(n_steps=warmup, pbar_max_steps=sim_dur+warmup, keep_data=False)

    while my_sim.curr_step < sim_dur + warmup:

        # # Set ramp metering rate.
        #if my_sim.curr_step % 50 / my_sim.step_length == 0:
        #     my_sim.set_tl_metering_rate(rm_id="crooswijk_meter", metering_rate=randint(1200, 2000))
        #     my_sim.set_tl_metering_rate(rm_id="a13_meter", metering_rate=randint(1200, 2000))
        
        # Step through n seconds.
        my_sim.step_through(n_seconds=n, pbar_max_steps=sim_dur+warmup)

        # if my_sim.curr_step == 100 / my_sim.step_length:
        #     my_sim.cause_incident(100, n_vehicles=2, edge_speed=5)

        # Check speed in merging area to decide if VSLs should be active or not (try to mimic what RWS does?)
        
        if my_sim.curr_step >= 50 / my_sim.step_length and useVSL:
            vsl1MergeSpeed = my_sim.get_interval_detector_data(detector_ids="vsl1SpeedRef", data_keys="speeds", n_steps=10, interval_end=0, avg_step_vals=True, avg_det_vals=True)
            if vsl1MergeSpeed < 80:
                my_sim.controllers["vsl1"].set_speed_limit(70)
            if vsl1MergeSpeed < 60:
                my_sim.controllers["vsl1"].set_speed_limit(50)
            if vsl1MergeSpeed > 100:
                my_sim.controllers["vsl1"].deactivate()

            # vsl2MergeSpeed = my_sim.get_interval_detector_data(detector_ids="a13_rm_downstream", data_keys="speeds", n_steps=10, interval_end=0, avg_step_vals=True, avg_det_vals=True)
            # if vsl2MergeSpeed < 80:
            #     my_sim.controllers["vsl2"].set_speed_limit(70)
            # if vsl2MergeSpeed < 60:
            #     my_sim.controllers["vsl2"].set_speed_limit(50)
            # if vsl2MergeSpeed > 100:
            #     my_sim.controllers["vsl2"].deactivate()
        # if my_sim.curr_step == 100 / my_sim.step_length:
        #     # Activate controllers & update UTSC phases.
        #     #my_sim.controllers["rerouter"].activate()z
        #     my_sim.controllers["vsl1"].set_speed_limit(50)
        #     my_sim.controllers["vsl2"].set_speed_limit(50)


            #my_sim.set_phases({"utsc": {"phases": ["GGrr", "yyrr", "rrGG", "rryy"], "times": [37, 3, 7, 3]}}, overwrite=False)

        # Deactivate controllers.
        # if my_sim.curr_step == 500 / my_sim.step_length:
        #     my_sim.controllers["vsl1"].deactivate()
        #     my_sim.controllers["vsl2"].deactivate()




    # End the simulation.
    my_sim.end()
    return my_sim #hopefully this is by reference?


if __name__ == "__main__":
    # Load environment variables for this machine
    load_dotenv()

    #Let's automate a few scenarios

    avShares=[0.25]
    vslStates=[True]

    for avShare in avShares:
        for vslState in vslStates:
            # Initialise the simulation object.
            my_sim = Simulation(scenario_name="A20_CIEM6220_SSMS_A20 VSL/AV scenario testing.")
            # Add "-seed {x}" to the command line to set a seed for the simulation
            sim_seed = "1" if "-seed" not in sys.argv[:-1] else sys.argv[sys.argv.index("-seed")+1]
            seed(int(sim_seed))

            runScenario(my_sim,useVSL=vslState)

            # Save the simulation data & print a summary, which is also saved.
            # my_sim.save_data("example_data.json") Let's not
            my_sim.save_data("ssms.pkl")
            my_sim.print_summary(save_file="ssms.txt")








    
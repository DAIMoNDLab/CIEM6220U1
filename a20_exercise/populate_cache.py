#Imports
from random import randint, seed
from dotenv import load_dotenv
import sys, pickle, shutil
from tud_sumo.simulation import Simulation
from tud_sumo.plot import Plotter
import numpy as np

#Globals
forceGUI=False
savesDir="./cache/"

#Custom functions

def runScenario(my_sim,useVSL=False,useRM=False):
    #Define some convenience dictionary to store vehicle ids and whether they are AVs or not
    vehicleClass={}
    # Start the simulation, defining the sumo config files. Add "-gui" to the command line to run with the GUI.
    my_sim.start("a20_exercise/a20_scenario/a20.sumocfg", get_individual_vehicle_data=False, gui=("-gui" in sys.argv) or forceGUI,
                 seed=sim_seed, units="metric", suppress_pbar=False) # Units can either be metric (km,kmph)/imperial (mi,mph)/UK (km,mph). All data collected is in these units.
    
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
    my_sim.add_tracked_junctions({"crooswijk_meter": {'meter_params': {'min_rate': 200, 'max_rate': 4000, 'queue_detector': "cw_ramp_queue"},
                                                    'flow_params': {'inflow_detectors': ["cw_ramp_inflow", "cw_rm_upstream"], 'outflow_detectors': ["cw_rm_downstream"]}},
                                "a13_meter": {'meter_params': {'min_rate': 200, 'max_rate': 4000, 'queue_detector': "a13_ramp_queue"},
                                                'flow_params': {'inflow_detectors': ["a13_ramp_inflow", "a13_rm_upstream"], 'outflow_detectors': ["a13_rm_downstream"]}}})
    
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
    #my_sim.save_objects("objects.json")

    # Add a function that is called on each new vehicle in the simulation. Simulation parameters are; curr_step,
    # vehicle_id, route_id, vehicle_type, departure, origin, destination. These values are filled automatically.
    # For other parameters, use a parameters dictionary as below. Use add_vehicle_out_funcs() for functions
    # called when vehicles exit the simulation (only with vehicle_id and/or curr_step). Vehicle in/out functions
    # can be removed using remove_vehicle_[in/out]_functions().

    # MR: wonder if I can use this to flexibly change the vehicle type on the fly? (yes)

    def stochastic_av_vehicle_class(simulation, vehicle_id, avShare=0):
        """
        Reassign vehicle ID to AV type if random roll < avShare
        """
        if simulation.vehicle_exists(vehicle_id):
            if randint(0,100)/100 < avShare:
                simulation.set_vehicle_vals(vehicle_id,type="AV")
                vehicleClass[vehicle_id]="AV"
            else:
                vehicleClass[vehicle_id]="HV"

    def set_BA_CACC_vehicle(simulation,vehicle_id,share=0):
        """
        Reassign vehicle ID to CACC_BA
        """
        if vehicleClass[vehicle_id] == "AV" and randint(0,100)/100 < share:
            simulation.set_vehicle_vals(vehicle_id,type="BA_AV")
        

    def set_BA_HV_vehicle(simulation,vehicle_id,share=0):
        """
        Reassign vehicle ID to HV_BA
        """
        if vehicleClass[vehicle_id] == "HV" and randint(0,100)/100 < share:
            simulation.set_vehicle_vals(vehicle_id,type="BA_cars")


    my_sim.add_vehicle_in_functions(stochastic_av_vehicle_class, parameters={"avShare": avShare})


    n, sim_dur, warmup = 1, 750 / my_sim.step_length, 0 / my_sim.step_length
    
    my_sim.set_tl_metering_rate(rm_id="crooswijk_meter", metering_rate=4000) #4000 should boil down to 'allow all'
    my_sim.set_tl_metering_rate(rm_id="a13_meter", metering_rate=4000)
    
    if warmup > 0:
        my_sim.step_through(n_steps=warmup, pbar_max_steps=sim_dur+warmup, keep_data=False)

    while my_sim.curr_step < sim_dur + warmup:

        
        
        # Step through n seconds.
        my_sim.step_through(n_seconds=n, pbar_max_steps=sim_dur+warmup)

        # if my_sim.curr_step == 100 / my_sim.step_length:
        #     my_sim.cause_incident(100, n_vehicles=2, edge_speed=5)

        # Check speed in merging area to decide if VSLs should be active or not (try to mimic what RWS does?)
        
        if my_sim.curr_step >= 50 / my_sim.step_length and (my_sim.curr_step % 50 / my_sim.step_length == 0) and useVSL:
            vsl1MergeSpeed = my_sim.get_interval_detector_data(detector_ids="vsl1SpeedRef", data_keys="speeds", n_steps=10, interval_end=0, avg_step_vals=True, avg_det_vals=True)
            if vsl1MergeSpeed < 80 and vsl1MergeSpeed > 60:
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

        # Set ramp metering rate.
        if useRM and (my_sim.curr_step % 50 / my_sim.step_length == 0): #Every 50 steps 
            #
            # You can input your controller logic here!
            #
            CWweavingSectionSpeed = my_sim.get_interval_detector_data(detector_ids="cw_rm_downstream", data_keys="speeds", n_steps=10, interval_end=0, avg_step_vals=True, avg_det_vals=True)
            if (CWweavingSectionSpeed < 130) :
                #Linearly metering more heavily as speed moves below threshold
                my_sim.set_tl_metering_rate(rm_id="crooswijk_meter", metering_rate=4000*(130-CWweavingSectionSpeed)/130)
            else:
                #Not metering at all
                my_sim.set_tl_metering_rate(rm_id="crooswijk_meter", metering_rate=4000)
            




    # End the simulation.
    my_sim.end()
    return vehicleClass  #hopefully this is by reference?

if __name__ == "__main__":
    # Load environment variables for this machine
    load_dotenv()

    #Let's automate a few scenarios

    avShares=np.around(np.linspace(0,1,11),2)
    vslStates=[False,True]
    rmStates=[False,True]

    counter=1
    total=(len(avShares))*len(vslStates)*len(rmStates)
    for avShare in avShares:
        for vslState in vslStates:
            for rmState in rmStates:
                print(f'Populating cache... scenario {counter}/{total}\n')
                counter=counter+1
                # Initialise the simulation object.
                my_sim = Simulation(scenario_name="A20_CIEM6220_"+str(avShare)+"_"+str(vslState)+"_"+str(rmState), scenario_desc="A20 VSL/AV Batch scenario testing.")
                # Add "-seed {x}" to the command line to set a seed for the simulation
                sim_seed = "1"# if "-seed" not in sys.argv[:-1] else sys.argv[sys.argv.index("-seed")+1]
                seed(int(1))
                try:
                    vClass=runScenario(my_sim,useVSL=vslState,useRM=rmState)
                except Exception as e:
                    print("Scenario computation failed.")
                    print(e)

                
                my_sim.save_data(savesDir+"A20_"+str(avShare)+"_"+str(vslState)+"_"+str(rmState)+".pkl")
                #my_sim.print_summary(save_file=savesDir+"A20_"+str(avShare)+"_"+str(vslState)+".txt") summary not needed, not really
                with open(savesDir+"A20_"+str(avShare)+"_"+str(vslState)+"_"+str(rmState)+"_vehTypeDict.pkl",'wb') as h: #rather, save the vehicle classes in their own named pickle file
                    pickle.dump(vClass,h)
                    #and move the ssm measurements into savesDir with the appropriate renaming as well
                shutil.copy("./a20_exercise/a20_scenario/a20ssm.xml",savesDir+"A20_"+str(avShare)+"_"+str(vslState)+"_"+str(rmState)+"_ssm.xml")

                
                










    
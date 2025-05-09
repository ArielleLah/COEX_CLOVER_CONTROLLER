# coex_clover_controller
To launch simulator run "roslaunch clover_simulation simulator.launch" in a terminal
To use smolagent type "roscd coex_clover_controller/scripts
                       python smoldroneagent.py" in a new terminal
To have objects within the world, in the gazebo simulation running click insert tab, add random 
objects onto the map and save as.  Ensure world is saved in ~/clover_simulation/resources/worlds/myworld.world
Then edit the simulator.launch file in /clover_simulation/launch/ Specifically change this line:
<arg name="world_name" value="$(find clover_simulation)/resources/worlds/my_world.world"/>

@Misc{smolagents,
 title = {`smolagents`: a smol library to build great agentic systems.},
 author = {Aymeric Roucher and Albert Villanova del Moral and Thomas Wolf and Leandro von Werra and Erik Kaunismäki},
 howpublished = {\url{https://github.com/huggingface/smolagents}},
 year = {2025}
}

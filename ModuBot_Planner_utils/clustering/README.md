## File Description
- main.py : the main file that needs to be executed, calls and uses code from other files listed.
- sim_tree.py : pybullet simulation, creation of trees and clusters in specific time steps.
- mst.py : file which creates graph, and corresponding minimum spanning tree.
- dabba.urdf : the block object.



## Executing the files 
In terminal </br>
The only file that needs to be executed is *main.py*. It takes a few arguments in the command line and they can be viewed by typing ``` python3 main.py -h ``` </br>
An example way of executing the file is given below.
```
python3 main.py --num_bots 8 --num_clusters 2 --seed 0 --debug True
```

**Debug mode**
We have added a debug mode to help better visualize and spot loopholes, by adding lines between nodes of a cluster and adding block ids on top its corresponding block. To enable debug mode pass ``--debug True`` at time of execution in terminal. 

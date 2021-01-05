**Note** - naming of files, functions and variables may not be very intuitive, please do read the comments that come prior to it.

## File Description
- sim_tree.py : pybullet simulation, creation of trees and clusters in specific time steps.
- dabba.urdf : the block object.
- main.py : the main file that needs to be executed, calls and uses code from other files listed.
- mst.py : file which creates graph, and corresponding minimum spanning tree.

### Understanding the code
We have tried to make code very readable and easy to use. The files contain alot of comments and the logic is very intuitive. We have added function descriptions which can be accessed by typing ``` help(function_name)```  in the file which imports it, press q to exit from the prompt. 

## Executing the files 
In terminal </br>
The only file that needs to be executed is *main.py*. It takes a few arguments in the command line and they can be viewed by typing ``` python3 main.py -h ``` </br>
An example way of executing the file is given below.
```
python3 main.py --num_bots 8 --num_clusters 2 --seed 0 --debug True
```

**Debug mode**
We have added a debug mode to help better visualize and spot loopholes, by adding lines between nodes of a cluster and adding block ids on top its corresponding block. To enable debug mode pass ``--debug True`` at time of execution in terminal. 

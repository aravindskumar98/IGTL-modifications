# IGTL-modifications
=====================

This provides a way to monitor for changes in parameters and communicates these changes in ROS2 Parameters to IGTL (This is using ROS as the server and pyIGTL as the client)


----------------------------------------------------------------------------------------------------------------------------------------

Build Instruction
-----------------

The following steps were tested on:

- Ubuntu 20.04 + ROS2 Galactic

First, install OpenIGTLink in your local computer. A detailed instruction can be found at http://openigtlink.org/. In the following instruction, we assume that the build directory for the OpenIGTLink library is located at: ~/igtl/OpenIGTLink-build

    $ cd <your OpenIGTLink directory>
    $ git clone https://github.com/openigtlink/OpenIGTLink.git
    $ mkdir OpenIGTLink-build
    $ cd OpenIGTLink-build
    $ cmake -DBUILD_SHARED_LIBS:BOOL=ON ../OpenIGTLink
    $ make

Install ROS 2 following [the ROS 2 Documentation](https://docs.ros.org/en/galactic/Installation.html). Then create your ROS workspace following the [documentation](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html) as follows:

    $ source /opt/ros/galactic/setup.bash
    $ mkdir -p ~/dev_ws/src
    $ cd ~/dev_ws/
    $ rosdep install -i --from-path src --rosdistro galactic -y # Make sure to resolve dependency
	
Then clone the current repository from GitHub into the src folder:

    $ cd ~/dev_ws/src
    $ git clone https://github.com/aravindskumar98/IGTL-modifications/ cpp_parameter_event_handler

and execute catkin_make in your workspace directory:

    $ cd ~/dev_ws/
    $ colcon build --cmake-args -DOpenIGTLink_DIR:PATH=<your OpenIGTLink directory>/OpenIGTLink-build

Note that sometimes it might not detect the bin folder. In that case, run:

    $ export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:<absolute path to your OpenIGTLink directory>/OpenIGTLink-build/bin 


References
----------
1. Frank T, Krieger A, Leonard S, Patel NA, Tokuda J. ROS-IGTL-Bridge: an open network interface for image-guided therapy using the ROS environment. Int J Comput Assist Radiol Surg. 2017 May 31. doi: 10.1007/s11548-017-1618-1. PubMed PMID: [28567563](https://www.ncbi.nlm.nih.gov/pubmed/?term=28567563).



    
    



# ROS2-Coordinates-Transformation

This project involves the use of 2 packages (**custom** and **function**) and a **launch file**:
- The first package "custom" includes a custom srv and msg:
  * The **custom srv** is needed to initialize the types of the json_file path and the matrix.
  * The **custom msg** is needed to initialize the types of the coordinates x, y, and z.
- The second package "function" includes 3 nodes:
  * **Node 1** acts as a publisher and publishes hard written coordinates to Node 2 using the custom msg.
  * **Node 3** acts as a service server to extract the matrix from the json file using the custom srv and sends it to its client Node 2.
  * **Node 2** acts as both a client and a subscriber where it recieves the coordinates from Node 1 and the matrix from Node 3. It then computes the newly transformed coordinates by applying matrix transformation and logs them. 
- Finally the **launch file** is used to launch all nodes together. So to run all nodes, it is sufficient to run the launch file only which launches the server first (node 3), then the listener_client (node 2), and finally the talker (node 1).
- Cmake files and xml files were also edited and managed to fix all dependencies available.

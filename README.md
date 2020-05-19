# Getting Started with ARCORE-Unity-Android Development

For ARCORE-Unity-Android Development refer to android branch.

# Getting Started with ROS-HoloLens communication
[ROS#](https://github.com/siemens/ros-sharp) is a open-source library that allows us to communicate with [ROS](https://www.ros.org/) from [Unity](https://unity.com/), a game engine that is used to develop augmented reality content such as Microsoft's [HoloLens](https://www.microsoft.com/en-us/hololens). 

This section is adapted from the [official page](https://github.com/siemens/ros-sharp) of the ROS# library. A detailed tutorial can be found [here](https://github.com/siemens/ros-sharp/wiki).

**Requirements**: Windows 10 Enterprise, Pro, or Education
- Windows 10 Home will NOT work because [Hyper-V](https://docs.microsoft.com/en-us/virtualization/hyper-v-on-windows/about/) is required for HoloLens Emulator.
- If you are from Stanford, you can obtain a free copy of Windows 10 Education from [here](https://stanford.onthehub.com/WebStore/Welcome.aspx). Sign in (top right) with your SUNet ID and search for "windows 10 education" in the top box. Add to cart and check out, you should obtain a product key.

*Note*: If you already have Windows 10 installed in your machine, you can simply upgrade to Windows 10 Education by manually entering in the product key as listed [here](https://docs.microsoft.com/en-us/windows/deployment/upgrade/windows-10-edition-upgrades#upgrade-by-manually-entering-a-product-key).

## Software installation
You can find an overview of the tools required over [here](https://docs.microsoft.com/en-us/windows/mixed-reality/install-the-tools). Alternatively, you may follow the steps below for a more detailed instruction.

### Installing Ubuntu OS / ROS on Windows using Hyper-V
1. [Turning on Hyper-V](https://docs.microsoft.com/en-us/virtualization/hyper-v-on-windows/quick-start/enable-hyper-v#enable-the-hyper-v-role-through-settings)
   - Enable developer mode on your PC at `Settings` > `Update & Security` > `For developers`. 
   - Under Windows search, type `Turn Windows Features on or off`.
   - Select `Hyper-V` and click `OK`.
   - You should be prompted to restart your Windows.
   
   **Note**: You need to have Hyper-V enabled to be able to run the HoloLens emulator. Hyper-V is also capable of running VMs such as Ubuntu OS. We recommend using Hyper-V to run Ubuntu OS because other VMs (e.g. Virtual Box / VMWare) do not work once Hyper-V is enabled. [Why?](https://seriouscodeblog.wordpress.com/2017/02/21/and-you-can-write-hololens-apps-in-macos-linux-too-part-1/)

2. Creating network switch
   - Open `Hyper-V Manager`. Under `Action` on the top tab, click on `Virtual Switch Manager`.
   - Create a `Internal` virtual switch and name it **InternalSwitch**. Proceed with the default settings.

3. Enabling internet connection in virtual machine
   - Under `Control Panel`, open `Network and Internet` then `Network and Sharing Center`.
   - Select an active internet connection and click on `Connections`.
   - Under `Properties`, click on the `Sharing` tab. `Allow other network users to connect through this computer's network connection` and select `vEthernet (InternalSwitch)` in Home networking connection.

4. [Installing Ubuntu on Hyper-V](https://docs.microsoft.com/en-us/virtualization/hyper-v-on-windows/quick-start/quick-create-virtual-machine)
   - Under Windows search, open up `Hyper-V Quick Create`.
   - Under operating system, select `Ubuntu 18.04`.
   - Under `More options`, change the network to `InternalSwitch`.
   - Click on `Create Virtual Machine`.
   - Remember the username and password that you set as it will be required later on.
   
   **IMPORTANT**: While you are setting up Ubuntu, **DO NOT** select the *Log In Automatically* option when creating your user account. If you are confused, check out this [link](https://www.zdnet.com/article/windows-10-tip-run-ubuntu-linux-in-an-enhanced-hyper-v-session/) for a screenshot. 
   
5. Starting your virtual machine
   - When you connect to Ubuntu, a prompt for the `Display configuration` will appear. Leaving it as **1366 by 768 pixels** is fine.
   - A *xrdp prompt* will appear. Under `Session`, select `Xorg`. The username and password is your Ubuntu's login credentials.

6. Setting up ROS on *Ubuntu*
   - Follow this [link](http://wiki.ros.org/melodic/Installation/Ubuntu) to install ROS on your **virtual machine**.

7. Configuring your ROS environment on *Ubuntu*
   - Follow this [link](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) to set up your `catkin_ws`.

8. Setting up [Gazebo](http://gazebosim.org/) on *Ubuntu*
   ```
   $ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
   $ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
   $ sudo apt-get update
   $ sudo apt-get install gazebo9
   $ sudo apt upgrade
   ```

9. Installing `rosbridge-suite` on *Ubuntu*
   ```
   $ sudo apt-get install ros-melodic-rosbridge-server
   ```

### Installing Visual Studios, HoloLens emulator, Unity and Mixed Reality Toolkit
1. If you are planning to use the HoloLens emulator, ensure that you have enabled Hyper-V as described in [Step 1](#software-installation).

2. Installing **Visual Studio 2019** on *Windows*
   - Download **Visual Studio 2019 Community** from [here](https://visualstudio.microsoft.com/downloads/).
   - During the installation, include the following workloads under `Desktop & Mobile`:
     - `Desktop development with C++`
     - `Universal Windows Platform development`
     - Within the UWP workload, check the following: `USB Device Connectivity`

3. Installing **HoloLens emulator (1st gen)** on *Windows*
   - Follow the instructions on this [page](https://docs.microsoft.com/en-us/windows/mixed-reality/using-the-hololens-emulator) to download and install the emulator.
   
4. [Testing](https://docs.microsoft.com/en-us/windows/mixed-reality/using-the-hololens-emulator#deploying-apps-to-the-hololens-emulator) the HoloLens emulator
   - Start Visual Studio 2019 and try to create a new `Holographic DirectX 11 App (Universal Windows)`. If it works, then you have all the required libraries in Visual Studios. Otherwise, add the proper workloads as stated [here](https://docs.microsoft.com/en-us/windows/mixed-reality/install-the-tools).
   - For HoloLens Emulator (1st gen), ensure that Platform is set to `x86` on the top bar. 
   - On the right of `x86`, there should be a green play button with a drop-down menu. Select `HoloLens Emulator` as the target device for debugging.
   - Click on the green play button to run the emulator.
   - If everything works, then the emulator is set up properly.
   
5. Installing **Unity** editor on *Windows*
   - [Download](https://unity3d.com/get-unity/download) and install Unity Hub.
   - Open up Unity Hub. Under `Installs` on the left, click `Add` and install the `2018.4.x` version.
   - Ensure that you select the following modules:
     - `UWP Build Support (IL2CPP)`
     - `UWP Build Support (.NET)`

6. Downloading [Mixed Reality Toolkit (MRTK)](https://github.com/microsoft/MixedRealityToolkit-Unity)
   - MRTK provides the basic building blocks for Unity development on HoloLens and includes a wide range of prefabs and scripts.
   - Go to [MRTK release page](https://github.com/Microsoft/MixedRealityToolkit-Unity/releases).
   - Under `Assets`, download:
     - `Microsoft.MixedRealityToolkit.Unity.Foundation.unitypackage`
     - **(Optional)** `Microsoft.MixedRealityToolkit.Unity.Extensions.unitypackage`
     - **(Optional)** `Microsoft.MixedRealityToolkit.Unity.Tools.unitypackage`
     - **(Optional)** `Microsoft.MixedRealityToolkit.Unity.Examples.unitypackage`

## Building your first HoloLens app and communicating with ROS

### Current set-up:
- Host OS: Windows 10 Education
- [Ubuntu 18.04](https://ubuntu.com/download/desktop) running on [Hyper-V](https://docs.microsoft.com/en-us/virtualization/hyper-v-on-windows/about/)
- [Unity 2018.4.x](https://unity3d.com/get-unity/download/archive)
- [Visual Studio 2019 Community](https://visualstudio.microsoft.com/downloads/)
- [Windows SDK 18362](https://developer.microsoft.com/en-us/windows/downloads/windows-10-sdk/)

*Note*: You'll have to complete the set-up described [above](#software-installation) before continuing with this section.

1. Starting a new Unity project
   - Open Unity Hub and click on `New Project`.
   - Leave the Template as `3D` and give your project any name you like. For example, we can name it as **box** for this demo.
   - Click on `Create` to start the project.
   
2. Modifying build settings in Unity
   - In Unity, under `File`> `Build Settings` > `Platform`, select `Universal Windows Platform` and use the following setup:
	 - Target Device: `Any device`
	 - Architecture: `x86`
	 - Build Type: `D3D`
	 - Target SDK Version: `10.0.18362.x`
	 - Minimum Platform Version: `10.0.10240.x`
	 - Visual Studio Version: `Latest installed`
	 - Build and Run on: `Local Machine`
	 - Build configuration: `Release`
   - Click on `Switch Platform`.
   
3. Modifying player settings in Unity
   - In Unity, under `Edit`> `Project Settings` > `Player`, select the Windows icon. 
   - Under `XR Settings`,
     - ensure `Virtual Reality Supported` is checked
     - Change `Depth Format` to `16-bit depth`
     - Change `Stereo Rendering Mode` to `Single Pass Instanced`
   - Under `Other Settings` > `Configuration`, select the following:
     - Scripting Runtime Version: `.NET 4.x Equivalent`
     - Scripting Backend: `IL2CPP`
   - Under `Publishing Settings` > `Capabilities`, ensure the following options are checked:
     - `InternetClientServer`
     - `PrivateNetworkClientServer`
     - `SpatialPerception`
     - `RemoteSystem`
   
4. Modifying audio settings in Unity
   - In Unity, under `Edit`> `Project Settings` > `Audio`, set the `Spatializer Plugin` to `MS HRTF Spatializer`.

5. Importing MRTK packages to your Unity project
   - Follow the instructions [here](https://microsoft.github.io/MixedRealityToolkit-Unity/Documentation/GettingStartedWithTheMRTK.html#import-mrtk-packages-into-your-unity-project).
   - We recommend only importing **Microsoft.MixedRealityToolkit.Unity.Foundation.unitypackage** for now to reduce the size of the file later on.

6. Importing ROS# (**UWP version**) to your Unity project
   - Download the file [here](https://github.com/dwhit/ros-sharp).
   - Unzip the file and you will find a package called `RosSharp` under `ros-sharp-master/Unity3D/Assets`. 
   - To put the `RosSharp` folder inside your Unity project, you can either drag and drop it into the Assets window in the Unity editor, or locate the file location of your project in your Windows File Explorer and place it there.
   
   **IMPORTANT**: Note that there is a difference between the [official version of ROS#](https://github.com/siemens/ros-sharp) and the [UWP version](https://github.com/siemens/ros-sharp). HoloLens is a UWP device.
   
7. Installing additional scripts to your Unity project
   - Download and extract this current [repository](https://github.com/armlabstanford/mixed_reality_setup).
   - Under `Unity scripts`, you should see 3 scripts.
   - Place the 3 scripts inside your Unity project under `Assets/RosSharp/Scripts/RosBridgeClient/RosCommuncation`.

8. Adding Mixed Reality Toolkit to the scene
   - On the top menu bar of your Unity editor, you should see `Mixed Reality Toolkit`.
   - Under this, click on `Add to Scene and Configure..`.

9. Modifying plugin in Ros Sharp
   - Download and extract this current [repository](https://github.com/armlabstanford/mixed_reality_setup).
   - Under `Plugins`, you should see two files: `Newtonsoft.Json` and `Newtonsoft.Json.dll`.
   - In your Unity project, under `Assets/RosSharp/Plugins`, replace the existing `Newtonsoft.Json` and `Newtonsoft.Json.dll` files with the ones you obtained from this repository.

10. Creating a Cube in the scene
    - Under the SampleScene on the left, right click and create a **3D Object** > **Cube**.
    - Click on `Cube` to select it.
    - The `Inspector` window is shown on the right. You should see properties such as `Transform`, `Cube (Mesh Filter)`, `Mesh Renderer` etc.
    - Under `Transform`, modify the size of the cube by changing the scale in the X, Y, Z directions to **0.2**.
    - Under `Transform`, shift the cube to the front of the camera by changing the location in the Z direction to **2**.
    - You may also wish to change the cube's color. Follow the instruction [here](http://mammothinteractive.com/changing-a-cubes-color-unity-tutorial/).
   
11. Adding Bounding Box and Manipulation Handler to your Cube in the scene
    - Ensure that `Cube` is still selected under the SampleScene on the left so that we are modifying the properties for the cube.
    - Under the `Inspector` window on the right, click on `Add Component` and add the following:
       - `Bounding Box`
       - `Manipulation Handler`
       
12. Adding RosConnector to your Unity scene
    - Under the SampleScene on the left, right click and `Create Empty`.
    - Right click on your newly created GameObject and rename it to **RosConnector**.
    - Click on `RosConnector`. On the right, under `Inspector`, you should only see `Transform` for now. Click on `Add Component` and add the following:
      - `Pose Publisher`
      - `Scale Publisher`
    - `Ros Connector` should be automatically added for you.
    - `Pose Publisher (Script)` and `Scale Publisher (Script)` are scripts that help to publish information from HoloLens to ROS. A separate script must be written for different ROS message types. In this case, `Pose Publisher (Script)` publishes a [Pose Message](https://docs.ros.org/api/geometry_msgs/html/msg/Pose.html). The topic that is published can be changed under `Topic`.

13. Adding the Cube's Transform to the publishers
    - Drag and drop your `Cube` from the SampleScene on the left to the `Published Transform` on the right under `Pose Publisher`. Do the same for `Scale Publisher`.
    - Modify your topic for `Pose Publisher` to be **/pose**. This is the topic that we subscribe to in ROS later on.
    - Modify your topic for `Scale Publisher` to be **/scale**. This is the topic that we subscribe to in ROS later on.
    - Under `Pose Publisher`, you should see the following now:
      - `Topic`: /pose
      - `Published Transform`: Cube (Transform)
    - Under `Scale Publisher`, you should see the following now:
      - `Topic`: /scale
      - `Published Transform`: Cube (Transform)

14. Finding the IP address of your Ubuntu machine
    - Find the IP address of your Ubuntu OS by using the `$ ifconfig` command in an Ubuntu terminal. If your Network Adapter was set-up correctly in [Step 4](#software-installation) during the installation phase, you should see the IP address listed under `eth0`. For example, it could be `inet 192.168.137.66`.
    
15. Modifying IP address under `ROSConnector` in Unity
    - Ensure that `RosConnector` is still selected under the SampleScene on the left so that we are modifying the properties for it.
    - Under `Inspector` on the right, copy and paste the IP address under `Ros Connector (Script)` > `Ros Bridge Server Url` and append `port 9090`. For example, it could be `ws://192.168.137.66:9090`.
    - `Ros Connector (Script)` is used to identify the IP address of our Ubuntu OS that is deployed in Hyper-V. 
    - Additionally under `Protocol`, ensure that `Web Socket UWP` is selected.
    - Under `Ros Connector (Script)`, you should see the following now:
      - `Timeout`: 10
      - `Serializer`: JSON
      - `Protocol`: Web Socket UWP
      - `Ros Bridge Server Url`: ws://192.168.137.66:9090 (**depending on your IP address**)
      
16. Building the HoloLens App from Unity
    - We are finally ready to build the app. Before exporting your app, remember to save your SampleScene in Unity so that we can continue modifying the app next time.
    - Under `File` > `Build Settings`, ensure `Universal Windows Platform` is selected as the build platform and click on `Build`. When the prompt for the file location appears, create a new folder called `App`, select it and click `Select Folder`.

17. Obtaining the sample ROS files
    - Download and extract this current [repository](https://github.com/armlabstanford/mixed_reality_setup).
    - Place the `box_demo` package in the `src` folder of your Catkin workspace in Ubuntu, then build by running `$ catkin_make` from the root folder of your catkin workspace. (Your catkin workspace is found in the home directory of the Ubuntu VM, and is usually called catkin_ws).
    - In the directory `box_demo/scripts` make the file `marker.py` executable by running
      ```
      chmod +x marker.py
      ```

18. Running ROS in Ubuntu
    - Open terminal in Ubuntu and run the following command:
      ```
      $ roslaunch box_demo box.launch
      ```

19. Running the HoloLens App
    - Under the `App` folder of your Unity project, you should now see a `demo.sln`. Double click on it to open it in Visual Studio.
    - In Visual Studio top menu bar, change to `Release`, `x86` and `HoloLens Emulator 10.0.x`. Then click on the green Play button located on the left of `HoloLens Emulator 10.0.x`.
    - Once the emulator is running, you should see `Client connected.  1 clients total.` in your Ubuntu terminal.
    - Use the arrow keys (to move user's gaze) and WASD (to walk around) in the emulator and find the box. Using your spacebar, you can pick the box and place it elsewhere by using the spacebar to drop it off. The location of the box in `rviz` should also update as you move the box around.

## Troubleshooting tips
**Q: I managed to build the HoloLens app and run it in the emulator but it is not connecting with ROS.**

**A**: Ensure that `Web Socket UWP` is selected in Unity (under your current scene > `ROSConnector` > `Protocol`). The original [ROS#](https://github.com/siemens/ros-sharp) is written for NET application whereas HoloLens is a UWP platform. Luckily, there is a UWP version of ROS# [here](https://github.com/dwhit/ros-sharp) which has been imported into the sample Unity `Box` project found in this repository. If you are interested, read this [thread](https://github.com/siemens/ros-sharp/issues/33) for more information.

**Q: My HoloLens client is connected to ROS but no message is being transmitted.**

**A**: This is a known issue faced by other users as well as seen in [here](https://github.com/siemens/ros-sharp/issues/201) and [here](https://github.com/siemens/ros-sharp/issues/33). The suspected reason is because of `Newtonsoft.Json` and `Newtonsoft.Json.dll` found under `\Box\Assets\RosSharp\Plugins`. 

A quick fix is proposed by other users:
- Download `JSON .NET For Unity` [here](https://assetstore.unity.com/packages/tools/input-management/json-net-for-unity-11347).
- Import it into your project folder, BUT only select `Newtonsoft.Json` and `Newtonsoft.Json.dll` under `AOT`.
- Copy and paste these 2 files (**NOT the .meta files**) from `Assets\JsonDotNet\Assemblies\AOT` folder into `\Assets\RosSharp\Plugins` folder.
- Delete the `JsonDotNet` folder under `Assets` to prevent multiple copies of `Newtonsoft.Json.XML` and `Newtonsoft.Json.dll`.

**NOTE**: You shouldn't have to do this if you followed the instruction for [Step 9](#building-your-first-hololens-app-and-communicating-with-ros).

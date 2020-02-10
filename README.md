# Setting Up ROS-Unity-Hololens
1. Requirements for using [Hololens](https://docs.microsoft.com/en-us/windows/mixed-reality/using-the-hololens-emulator) 1 or 2 emulator:
   - **Requirements**: Windows 10 Enterprise, Pro, or Education
	- Windows 10 Home will NOT work because [Hyper-V](https://docs.microsoft.com/en-us/virtualization/hyper-v-on-windows/about/) is required for Hololens Emulator.
	- If you are from Stanford, you can obtain a free copy of Windows 10 Education from [here](https://stanford.onthehub.com/WebStore/Welcome.aspx). Sign in (top right) with your SUNet ID and search for "windows 10 education" in the top box. Add to cart and check out, you should obtain a product key.

Note: If you already have Windows 10 installed in your machine, you can simply upgrade to Windows 10 Education by manually entering in the product key as listed [here](https://docs.microsoft.com/en-us/windows/deployment/upgrade/windows-10-edition-upgrades#upgrade-by-manually-entering-a-product-key).

2. [Turning on Hyper-V](https://docs.microsoft.com/en-us/virtualization/hyper-v-on-windows/quick-start/enable-hyper-v#enable-the-hyper-v-role-through-settings)
   - Under Windows search, type **Turn Windows Features on or off**.
   - Select **Hyper-V** and click **OK**.
   - You should be prompted to restart your Windows.
   -  You need to have Hyper-V enabled to be able to run the emulator so do not use a virtual machine. It will not work. [Why?](https://seriouscodeblog.wordpress.com/2017/02/21/and-you-can-write-hololens-apps-in-macos-linux-too-part-1/)
   -  To enable Hyper-V go to windows search and type: "Turn Windows features on or off" and enable Hyper-V
   -  Make sure to enable virtualization in the BIOS of your windows machine.
		
3. Follow tool instructions one by one. If one of them fails there is something going wrong: 
   -  https://docs.microsoft.com/en-us/windows/mixed-reality/install-the-tools
   -  Once you install the emulator successfully, go to visual studio 2019. Getting to this point is the hardest part. Create a new project using DirectX 11 as shown below:
   <a href="https://ibb.co/vDWKZLj"><img src="https://i.ibb.co/qx8Pkmr/VS2019.png" alt="VS2019" border="0"></a>
   - If the option does not appear you are missing libraries. Launch the Visual studio installer and add the proper workloads. Refer to the [link](https://docs.microsoft.com/en-us/windows/mixed-reality/install-the-tools) in the **Visual Studio Workloads** section. 
   - Follow the instructions of these [video](https://www.youtube.com/watch?v=0ImaZ_Aqe3I) Minute 2:00 to pull up the emulator.
		
		
4. When installing Unity use the 2018.4 version to avoid conflicts: 
   - Install [Unity](https://unity3d.com/unity/qa/lts-releases?version=2018.4)
   - Install [Unity Hub](https://unity3d.com/get-unity/download ). Good interface to install dependencies: 
   - Get [Unity setup for hololens](https://www.youtube.com/watch?v=OYx4qIqi0oI). The video does not show you everything, but the narrator does explain the process properly. **NOTE:** UWP = Windows Store
   - Install [Mixed Reality Toolkit](https://github.com/Microsoft/MixedRealityToolkit-Unity/releases) packages:
   - Create a new project in unity.
	- In Unity select Assets, Import Package, Custom package (selected the downloaded package)
	- Follow this strict order else it will not work:
			Under Assets, [download](https://github.com/microsoft/MixedRealityToolkit-Unity/releases/tag/v2.2.0):
      - Microsoft.MixedRealityToolkit.Unity.Foundation.unitypackage
      - (Optional) Microsoft.MixedRealityToolkit.Unity.Extensions.unitypackage
      - (Optional) Microsoft.MixedRealityToolkit.Unity.Tools.unitypackage
      - (Optional) Microsoft.MixedRealityToolkit.Unity.Examples.unitypackage
      - (Optional, ExperimentalMicrosoft.MixedRealityToolkit.Unity.Providers.UnityAR.unitypackage
	
			
   - For unity tutorials I recommend this [series](https://www.youtube.com/watch?v=pTLCMZ_qvTw&list=PLGmYIROty-5bpzKQNK3mRMi4pmh_LinV4&index=1).



## Getting Started with ARCORE-Unity-Android Development

For ARCORE-Unity-Android Development refer to android branch.


## Getting started with [ROS#](https://github.com/siemens/ros-sharp)
[ROS#](https://github.com/siemens/ros-sharp) is a open-source library that allows us to communicate with [ROS](https://www.ros.org/) from [Unity](https://unity.com/), a game engine that is used to develop augmented reality content such as Microsoft's [Hololens](https://www.microsoft.com/en-us/hololens). 

This section is adapted from the [official page](https://github.com/siemens/ros-sharp) of the ROS# library. A detailed tutorial can be found [here](https://github.com/siemens/ros-sharp/wiki).

**Requirements**: Windows 10 Enterprise, Pro, or Education
- Windows 10 Home will NOT work because [Hyper-V](https://docs.microsoft.com/en-us/virtualization/hyper-v-on-windows/about/) is required for Hololens Emulator.
- If you are from Stanford, you can obtain a free copy of Windows 10 Education from [here](https://stanford.onthehub.com/WebStore/Welcome.aspx). Sign in (top right) with your SUNet ID and search for "windows 10 education" in the top box. Add to cart and check out, you should obtain a product key.

Note: If you already have Windows 10 installed in your machine, you can simply upgrade to Windows 10 Education by manually entering in the product key as listed [here](https://docs.microsoft.com/en-us/windows/deployment/upgrade/windows-10-edition-upgrades#upgrade-by-manually-entering-a-product-key).

1. [Turning on Hyper-V](https://docs.microsoft.com/en-us/virtualization/hyper-v-on-windows/quick-start/enable-hyper-v#enable-the-hyper-v-role-through-settings)
   - Under Windows search, type **Turn Windows Features on or off**.
   - Select **Hyper-V** and click **OK**.
   - You should be prompted to restart your Windows.

2. Creating network switch
   - Open **Hyper-V Manager**. Under **Action** on the top tab, click on **Virtual Switch Manager**.
   - Create a **Internal** virtual switch and name it **InternalSwitch**. Proceed with the default settings.

3. Enabling internet connection in virtual machine
   - Under **Control Panel**, open **Network and Internet** then **Network and Sharing Center**.
   - Select an active internet connection and click on **Connections**.
   - Under **Properties**, click on the **Sharing** tab. **Allow other network users to connect through this computer's network connection** and select **vEthernet (InternalSwitch)** in Home networking connection.

4. [Installing Ubuntu on Hyper-V](https://docs.microsoft.com/en-us/virtualization/hyper-v-on-windows/quick-start/quick-create-virtual-machine)
   - If you prefer to install a VM such as  instead of using Hyper-V, skip to *Step 6*.
   - Under Windows search, open up **Hyper-V Quick Create**
   - Under operating system, select **Ubuntu 18.04**.
   - Under **More options**, change the network to **InternalSwitch**.
   - Click on **Create Virtual Machine**.
   - Remember the username and password that you set as it will be required later on.
   
   **IMPORTANT**: While you are setting up Ubuntu, **DO NOT** select the *Log In Automatically* option when creating your user account. If you are confused, check out this [link](https://www.zdnet.com/article/windows-10-tip-run-ubuntu-linux-in-an-enhanced-hyper-v-session/) for a screenshot. 
   
5. Starting your virtual machine
   - When you connect to Ubuntu, a prompt for the **Display configuration** will appear. Leaving it as **1366 by 768 pixels** is fine.
   - A *xrdp prompt* will appear. Under **Session**, select **Xorg**. The username and password is your Ubuntu's login credentials.
   - Proceed to Step 7.

6. Installing a VM instead of using Hyper-V
   - A popular open source Virtual Machine (VM) is Oracle's [VirtualBox](https://www.virtualbox.org/) that can be downloaded for free. Alternatively, Stanford provides another VM known as [VMware](https://www.vmware.com/) (Fusion on Mac, Workstation on Windows/Linux). To install VMWare, go to this [link](https://stanford.onthehub.com/WebStore/Welcome.aspx) and sign in (top right) with your SUNet ID. Search for "VMware" in the top box and download the respective version for your OS.
   - To install Ubuntu, a guide for VirtualBox can be found [here](https://brb.nci.nih.gov/seqtools/installUbuntu.html).

7. Setting up ROS on Ubuntu
   - Follow this [link](http://wiki.ros.org/melodic/Installation/Ubuntu) to install ROS on your virtual machine.

8. Configuring your ROS environment
   - Follow this [link](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) to set up your `catkin_ws`.

9. Setting up [Gazebo](http://gazebosim.org/)
```
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install gazebo9
$ sudo apt upgrade
```

10. Installing `rosbridge-suite`
   ```
   $ sudo apt-get install ros-kinetic-rosbridge-server
   $ git clone https://github.com/siemens/ros-sharp.git ~/Desktop/ros-sharp
   ```
   - Place the `file_server` package (found under `Desktop/ros-sharp/ros`) in the `src` folder of your Catkin workspace, then build by running `$ catkin_make` from the root folder of your catkin workspace. (Your catkin workspace is found in the home directory of the Ubuntu VM, and is usually called catkin_ws).

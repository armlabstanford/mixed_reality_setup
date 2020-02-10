# Setting Up ROS-Unity-Hololens


## Getting Started with ARCORE-Unity-Android Development

This tutorial will guide you through how to develop apps based on Unity for Android Arcore compatible devices to emulate AR Objects. At the end of this tutorial you should be able to control objects as show in the video below:
[![Box Manipulation Demo in ARcore-Unity-Android](https://i9.ytimg.com/vi/3wR_BDyft5M/mq2.jpg?sqp=CJ2o5PEF&rs=AOn4CLA5eSE2bn3DY0l_oPv4MU5-oWprxw)](https://youtu.be/3wR_BDyft5M "Box Manipulation Demo in ARcore-Unity-Android")


## Getting started with [ROS#](https://github.com/siemens/ros-sharp)
[ROS#](https://github.com/siemens/ros-sharp) is a open-source library that allows us to communicate with [ROS](https://www.ros.org/) from [Unity](https://unity.com/), a game engine that is used to develop augmented reality content such as Microsoft's [Hololens](https://www.microsoft.com/en-us/hololens). 

This section is adapted from the [official page](https://github.com/siemens/ros-sharp) of the ROS# library. A detailed tutorial can be found [here](https://github.com/siemens/ros-sharp/wiki).

**Requirements**: Windows 10 Enterprise, Pro, or Education
- Windows 10 Home will NOT work because [Hyper-V](https://docs.microsoft.com/en-us/virtualization/hyper-v-on-windows/about/) is required for Hololens Emulator.
- If you are from Stanford, you can obtain a free copy of Windows 10 Education from [here](https://stanford.onthehub.com/WebStore/Welcome.aspx). Sign in (top right) with your SUNet ID and search for "windows 10 education" in the top box. Add to cart and check out, you should obtain a product key.

Note: If you already have Windows 10 installed in your machine, you can simply upgrade to Windows 10 Education by manually entering in the product key as listed [here](https://docs.microsoft.com/en-us/windows/deployment/upgrade/windows-10-edition-upgrades#upgrade-by-manually-entering-a-product-key).

1. Installation of Linux OS on VM

   - A popular open source Virtual Machine (VM) is Oracle's [VirtualBox](https://www.virtualbox.org/) that can be downloaded for free. Alternatively, Stanford provides another VM known as [VMware](https://www.vmware.com/) (Fusion on Mac, Workstation on Windows/Linux). To install VMWare, go to this [link](https://stanford.onthehub.com/WebStore/Welcome.aspx) and sign in (top right) with your SUNet ID. Search for "VMware" in the top box and download the respective version for your OS.

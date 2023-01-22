# charged-up-2023

# April Tags

> ! Before doing anything with apriltags make sure to go to the standard tab and change the family to AprilTag Classic 16h5


> There are 3 levels of 3D apriltag tracking:
  Point of interest,
  Full 3D tracking
  and Robot Localization.
* > Point of interest tracking:
    Allows you to create an offset from the location of the april tag, 
    going to `Advanced > 3D point-of-interest` 
    you will find 3 numbers you can change to create an offset
    for example, if your point of interest is above the april tag you can change the y value to be above,
    Be aware the april tag is tracked upside down so if you have something above 
    you are going to have to make it a negative number.
    ![pointOfInterest](https://user-images.githubusercontent.com/43415244/214322731-569b51d0-d423-44c8-9ec7-24f395d211d9.png)

* > Full 3D tracking:
    found under `3D point-of-interest` in the `Advanced` tab you will find `MegaTag Field-Space-Localization`
    which will have every transform, these transforms are where the April tags are relative to the robot
    under `MegaTag Field-Space-Localization` you can find a 3D view where you can see the robot and the apriltags so you can change the transforms and see what changes 
    ![FieldLocalization](https://user-images.githubusercontent.com/43415244/214322910-15078ca5-03d8-4565-b60c-83717addfcda.png)
    ![Vizualizer](https://user-images.githubusercontent.com/43415244/214323144-3268fe4c-815e-48c6-8236-911523b3ae11.png)

* > Robot Localization:
    I didn't really understand what the `Robot Localization` does but from my understanding it guesses the robots location using the location of the apriltags    
    
# Notes     

>these changes were recommended by Limelight's Documentation and briefly tested by me.
> In the `input` tab
  * Change `Pipeline Type` to `Fiducial Markers`
  * setting the `Black-level offset` to `0`
  * setting `Sensor Gain` to `20`
  * reducing `Exposure` to reduce motion blur, I have found that `185` works best in the schools lighting but we will probably have to change it in the compatition anyways.
  * increasing the resolution can effect range and accuracy, I have found that 640x480 at 90fps works the best for our use.



> The Limelight should be mounted above or bellow the Apriltag
  and should not be looking directly at the Apriltag but angled down or up.

![inputTab](https://user-images.githubusercontent.com/43415244/214323246-1620b4b4-45a7-4c50-8e97-17822d4753df.png)
> In the `Standard` tab
  - you can also increase framerate by increasing `Detector Downscale`.
  - you can filter IDs to make the limelight check for a specific ID by puttin the ID number you want to remove in the text box next to the `ID filters`.
  
![image](https://user-images.githubusercontent.com/43415244/214323531-a70001ee-ecdb-4d9e-b536-f8c6b2b0463b.png)


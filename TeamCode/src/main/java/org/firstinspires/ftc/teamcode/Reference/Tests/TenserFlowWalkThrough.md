INFO from https://youtu.be/LGSL5izjCKU

Go to https://ftc-ml.firstinspires.org
Team Shipping Element (TSE) can be a card board box painted a vibrant color with multipule colors, stipes, patterns, ect.
Min - 3x3x4
Max - 4x4x8

When taking videos take it at the same postition as where you will be starting at during auto
Record with different backrounds, lighting conditions, and different TSE positions
When Recording it has to be in different starting positions for the robot (Red side, Warehouse side)
Include shots without TSE
When taking the video as you move it pause the video
Video length should be no more than 20 seconds (Can be more but will make us lose hours we can work on it)

Upload video to FTC website
Click on video
Draw bounding box over TSE
Make sure to label it TSE (Very Important! If one is incorrect the whole thing will break!)
Draw the bounding box for about 5 more
Press the start tracking button to speed up the procces
After a major change in the video it might lose sight of the TSE but say it is still there; You will have to change it.

When done it will save by it self
Go to home page
Select video
Click on "Produce Dataset"
Description is the Name (Does not have to be TSE; "TSE Video" is good)
Go to the tab "Datasets"
Click on the data set you made
Click "Start Training"
Change the training steps so that the epochs can be around 100
You can see the epochs at the bottom

Once the model is done training go to the Models tab
Click on the model
Click "Download Model"

Connect to robot
Search up "http://192.168.43.1/" on google
Go to Manage
Scroll sown to "Upload TenserFlow Lite Model File"
Upload the ".tflite" file
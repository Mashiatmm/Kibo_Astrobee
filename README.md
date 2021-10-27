# Kibo_Astrobee

Competition : https://iss.jaxa.jp/krpc/1st/index.html

We used zxing library for reading QR Tags and opencv library to read AR Tags.

Our movetoPos function moves Astrobee to each QR code's position iteratively while checking and avoiding obstacles with the help of other two functions : CheckforCollision and Obstacle.
After reaching each QR Code's position, the QR code is read with the code written inside readQRCode function.

The movetoP3 function first avoids obstacles (calling checkforCollision and obstacle functions) then move to the point P3. 
The readArucoMarker function tries to read the AR Tag from the P3 position.

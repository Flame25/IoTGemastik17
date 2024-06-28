# 1) load the model
from tensorflow import keras
model = keras.models.load_model('efficientnetb3-Plant.h5')
model.load_weights('weights.h5')

import cv2
import numpy as np 
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db

cred = credentials.Certificate("embedded-3d80d-firebase-adminsdk-ktn53-539689387b.json")
firebase_admin.initialize_app(cred, {"databaseURL" : "https://embedded-3d80d-default-rtdb.asia-southeast1.firebasedatabase.app/"})

ref = db.reference("/label")

# Open the device at the ID 0
# Use the camera ID based on
# /dev/videoID needed
cap = cv2.VideoCapture(0)

def __draw_label(img, text, pos, bg_color):
   font_face = cv2.FONT_HERSHEY_SIMPLEX
   scale = 0.4
   color = (0, 0, 0)
   thickness = cv2.FILLED
   margin = 2
   txt_size = cv2.getTextSize(text, font_face, scale, thickness)

   end_x = pos[0] + txt_size[0][0] + margin
   end_y = pos[1] - txt_size[0][1] - margin

   cv2.rectangle(img, pos, (end_x, end_y), bg_color, thickness)
   cv2.putText(img, text, pos, font_face, scale, color, 1, cv2.LINE_AA)

#Check if camera was opened correctly
if not (cap.isOpened()):
    print("Could not open video device")

# 2) fetch one frame at a time from your camera
while(True):
    
    # frame is a numpy array, that you can predict on 
    ret, frame = cap.read()
    
    # 3) obtain the prediction
    # depending on your model, you may have to reshape frame
    frame_resized = cv2.resize(frame,(224,224))
    frame_resized= frame_resized[np.newaxis, :, :, :]
    prediction = model(frame_resized, training=False)
    # you may need then to process prediction to obtain a label of your data, depending on your model. Probably you'll have to apply an argmax to prediction to obtain a label.
    
    # Result of Pred 
    pred_res = np.argmax(prediction)
    ref.set(int(pred_res))
    # 4) Adding the label on your frame
    __draw_label(frame, 'Label: {}'.format(pred_res), (20,20), (255,0,0))

    # 5) Display the resulting frame
    cv2.imshow("preview",frame)
   
    #Waits for a user input to quit the application
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
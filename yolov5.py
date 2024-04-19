import torch
import cv2

# Load the YOLOv5 model
model = torch.hub.load('ultralytics/yolov5', 'custom', path='yolov5s.pt')
model.eval() 

# Open a connection to the camera (0 corresponds to the default camera)
cap = cv2.VideoCapture(0)

while True:
    # Read a frame from the camera
    ret, frame = cap.read()
    
    if not ret:
        break
    
    # Perform object detection on the frame
    results = model(frame)
    
    # Get the detected objects' information
    detections = results.pred[0]
    
    # Draw bounding boxes around detected objects and display label and accuracy
    for detection in detections:
        bbox = detection[:4].cpu().numpy()  # Get bounding box coordinates
        class_id = int(detection[5])        # Get predicted class id
        confidence = detection[4].item()    # Get confidence score
        
        if confidence > 0.1:  # Adjust this threshold as needed
            x, y, w, h = bbox
            x, y, w, h = int(x), int(y), int(w), int(h)
            
            # Draw bounding box
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # Get class label and accuracy from the model
            class_label = model.names[class_id]
            accuracy = confidence * 1
            
            # Put class label and accuracy on the image
            label = f'Label: {class_label}, Accuracy: {accuracy:.2f}%'
            cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    # Display the resulting frame
    cv2.imshow('Real-time Object Detection', frame)
    
    # Press 'q' to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()
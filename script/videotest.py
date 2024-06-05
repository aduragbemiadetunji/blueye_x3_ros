# import cv2
# import customtkinter as ctk
# import threading
# from PIL import Image, ImageTk

# class BlueyeGUI:
#     def __init__(self, root):
#         self.root = root
#         self.root.title("Blueye X3 Control Interface")
        
#         # Add other GUI elements here
#         # For example, add control buttons, sliders, etc.

#         # Frame to display the video feed
#         self.video_frame = ctk.CTkFrame(root)
#         self.video_frame.pack()

#         self.label = ctk.CTkLabel(self.video_frame)
#         self.label.pack()

#         # Start the video feed in a separate thread
#         self.video_thread = threading.Thread(target=self.update_video_feed)
#         self.video_thread.daemon = True
#         self.video_thread.start()

#     def update_video_feed(self):
#         # Define GStreamer pipeline
#         gst_pipeline = (
#             "rtspsrc location=rtsp://192.168.1.101:8554/test latency=0 ! "
#             "rtph264depay ! avdec_h264 ! videoconvert ! appsink"
#         )

#         cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
#         if not cap.isOpened():
#             print("Error: Unable to open camera feed")
#             return

#         while cap.isOpened():
#             ret, frame = cap.read()
#             if not ret:
#                 break

#             # Convert the frame to RGB
#             frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
#             # Convert the frame to ImageTk format
#             img = Image.fromarray(frame)
#             imgtk = ImageTk.PhotoImage(image=img)
            
#             # Update the label with the new frame
#             self.label.imgtk = imgtk
#             self.label.configure(image=imgtk)

#         cap.release()

# if __name__ == "__main__":
#     root = ctk.CTk()
#     gui = BlueyeGUI(root)
#     root.mainloop()


import cv2
print(cv2.getBuildInformation())

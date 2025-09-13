# UAV Object Detection & Approximate GPS Localization

This project demonstrates how to use **YOLOv8 (OpenVINO optimized)** with a UAV camera feed to:
- Detect objects in real-time
- Estimate the **approximate GPS coordinates** of the detected object using only UAV onboard **GPS and camera** data

⚠️ **Note:** The GPS localization part is only an approximation (not a precise georeferencing system).  
It was developed as a proof-of-concept for UAV research.

---

## 🚀 Features
- Real-time object detection with **YOLOv8 + OpenVINO**
- Multi-threaded frame reading & processing (reduces lag)
- Distance estimation (pixel → real-world rough conversion)
- Approximate GPS coordinate calculation based on UAV position, pitch, yaw, and altitude

---

## 📂 Project Structure
```
.
├── UAV-YOLO-GPS.py        # Main script (YOLO inference + GPS estimation)
└── README.md      # Project description
```

---

## ⚙️ Requirements
- Python 3.8+
- OpenCV
- Ultralytics (YOLOv8)
- OpenVINO runtime (for optimized inference)

Install dependencies:
```bash
pip install ultralytics opencv-python
```

---

## ▶️ Usage
1. Clone this repo:
   ```bash
   git clone https://github.com/MuhammetAliKerbali/UAV-YOLO-GPS.git
   cd UAV-YOLO-GPS
   ```

2. Run the main script:
   ```bash
   python main.py
   ```

3. Press **`q`** to exit.

---

## 🛰️ How It Works
1. **YOLOv8 Model** detects objects on the UAV camera feed  
2. **Frame center & object center** difference is calculated  
3. Using UAV’s **GPS + altitude + pitch + yaw**, the script computes an **approximate offset**  
4. Final output is an **estimated latitude & longitude** of the detected object

---

## 📌 Example
- Detected objects are highlighted with bounding boxes  
- Approximate distance & GPS coordinate calculations are logged/drawn on the frame  

---

## ⚠️ Limitations
- GPS estimation is **rough** and may have large errors  
- Camera calibration is not applied (so pixel → meter scaling is approximate)  
- Works best for **demo and research**, not real missions  

---

## 📜 License
This project is open-source under the **MIT License**. Feel free to modify and use it.

---

👨‍💻 Developed as part of a UAV research project.

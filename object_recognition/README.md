### Object Recognition Using YOLOv3/TinyYOLOv3 in ROS

This package contains 3 scripts with the following publishers and subscribers:

**Subscribers**: 

* `[topic]/image_raw`: Image data from camera node.

**Publishers**:

* `object_detector`: Publishes label and prediction of the class with maximum score and coordiates of the bounding box, under the message type `Predictor.msg`
* `object_detector/image_raw`: Publishes image data with bounding boxes, labels and scores.

Note: `classify_tinyyolov3_lite.py`  does not publish bounding box and image information. 

#### Installation:

##### Dependencies:

* Install Tensorflow version 1.15 (requires CUDA 10.0)

  ```bash
  $ pip install --user --upgrade tensorflow-gpu=1.15
  ```

  If not training and facing troubles with CUDA installation, then install the CPU only version. 

  ```bash
  $ pip install --user --upgrade tensorflow=1.15
  ```

  Ensure the system python path still points to python2.7

* Install Keras:

  Install Keras version 2.3.1

  ```bash
  $ pip install keras==2.3.1
  ```

  Install h5py. You will need this to load the models.

  ```bash
  $ pip install h5py
  ```


* Numpy: 

  ```bash
  $ pip install numpy
  ```

* OpenCV

**Model files:**

Download pre-trained models (`.h5`) for TinyYOLOv3 and YOLOv3 trained on the MSCOCO dataset.

https://drive.google.com/drive/folders/1ni9L2r2xKmSfoaX6T8I2Biy53taakAn-?usp=sharing

Copy the model files into the `models` folder.
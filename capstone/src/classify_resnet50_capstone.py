#!/usr/bin/env python3
import numpy as np
import tensorflow as tf
from keras.applications.resnet import ResNet50, preprocess_input, decode_predictions


class ClassifyResnet50(object):
    def __init__(self):
        # Tensorflow and Keras setup
        GPU_OPTIONS = tf.compat.v1.GPUOptions(allow_growth=True)
        CONFIG = tf.compat.v1.ConfigProto(gpu_options=GPU_OPTIONS)
        CONFIG.gpu_options.per_process_gpu_memory_fraction = 0.5

        sess = tf.compat.v1.Session(config=CONFIG)
        tf.compat.v1.keras.backend.set_session(sess)
        self.model = ResNet50(weights='imagenet')

    def classify(self, img):
        np_image = np.asarray(img)  # read as np array
        np_image = np.expand_dims(np_image, axis=0)  # Add another dimension for tensorflow
        np_image = np_image.astype(float)  # preprocess needs float64 and img is uint8
        np_image = preprocess_input(np_image)  # Regularize the data

        preds = self.model.predict(np_image)  # Classify the image
        pred_string = decode_predictions(preds, top=1)  # Decode top 1 predictions
        return pred_string

    def find_traffic_light(self, prediction):
        if prediction == 'traffic_light':
            return True
        return False

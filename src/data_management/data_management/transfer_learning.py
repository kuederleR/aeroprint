from tensorflow.keras.applications import EfficientNetV2L
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Conv2D, GlobalAveragePooling2D, Dense, Dropout

model = EfficientNetV2L(weights='imagenet', 
                        include_top=False,
                        input_shape=(224, 224, 3))

for layer in model.layers:
    layer.trainable = False

my_model = Sequential([model, 
                      Conv2D(1024, 3, 1, activation='relu'),
                      GlobalAveragePooling2D(),
                      Dense(1024, activation='relu'), 
                      Dropout(0.2),
                      Dense(1024, activation='relu'), 
                      Dropout(0.2),
                      Dense(1, activation='sigmoid')])
# For more than two classifications, change the last Dense layer to the number of classes and the activation to 'softmax'
import os
try: 
   os.mkdir('images')
except: pass

try:
   os.makedirs(os.path.join('images', 'cats'))
except: pass

try:
   os.makedirs(os.path.join('images', 'dpgs'))
except: pass





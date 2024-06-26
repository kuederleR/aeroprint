from tensorflow.keras.applications import EfficientNetV2L

model = EfficientNetV2L(weights='imagenet', 
                        include_top=False,
                        input_shape=(224, 224, 3))

for layer in model.layers:
    layer.trainable = False

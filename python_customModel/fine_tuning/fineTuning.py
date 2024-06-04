import os
import tensorflow as tf
from keras.preprocessing.image import ImageDataGenerator

#Cartella root per il training
train_dir = "train"
samuele_dir = os.path.join(train_dir, "samuele")
manuel_dir = os.path.join(train_dir, "manuel")
sconosciuto_dir = os.path.join(train_dir, "sconosciuto")

#Definizione dei parametri per il training
input_shape = (224, 224)
batch_size = 32
epochs = 10
num_classes = 3 

train_datagen = ImageDataGenerator(
    rescale=1./255,
    rotation_range=20,
    width_shift_range=0.2,
    height_shift_range=0.2,
    shear_range=0.2,
    zoom_range=0.2,
    horizontal_flip=True,
    validation_split=0.2  #Suddividiamo il dataset in 80% training e 20% validation
)

train_generator = train_datagen.flow_from_directory(
    train_dir,
    target_size=input_shape,
    batch_size=batch_size,
    class_mode='categorical',
    subset='training'
)

validation_generator = train_datagen.flow_from_directory(
    train_dir,
    target_size=input_shape,
    batch_size=batch_size,
    class_mode='categorical',
    subset='validation'
)

#Carichiamo il modello pre addestrato MobileNetV2
base_model = tf.keras.applications.MobileNetV2(input_shape=input_shape + (3,), weights='imagenet', include_top=False)

#Congelamento dei pesi del modello
base_model.trainable = False

global_average_layer = tf.keras.layers.GlobalAveragePooling2D()(base_model.output)
output_layer = tf.keras.layers.Dense(num_classes, activation='softmax')(global_average_layer)

model = tf.keras.models.Model(inputs=base_model.input, outputs=output_layer)
model.compile(optimizer='adam', loss='categorical_crossentropy', metrics=['accuracy'])

#Esecuzione del fine tuning del modello
history = model.fit(
    train_generator,
    epochs=epochs,
    validation_data=validation_generator
)

#Salvataggio del modello in formato saved_model
model.save("saved_model")
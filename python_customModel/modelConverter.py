import tensorflow as tf

#Caricamento del modello e conversione
converter = tf.lite.TFLiteConverter.from_saved_model("saved_model")
converter.optimizations = [tf.lite.Optimize.DEFAULT]
#Quantizazione dei pesi
quantized_model = converter.convert()

#Salvataggio del modello
with open('quantized_model.tflite', 'wb') as f:
    f.write(quantized_model)


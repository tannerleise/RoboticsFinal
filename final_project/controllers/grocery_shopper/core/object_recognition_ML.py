import os
from PIL import Image
from controller import Camera
##insert following code to behaviors to test##

# while robot.robot.step(robot.timestep) != -1:
#     # grab_sequence.tick_once()
#     if(timer == 5):
#         t_obj = robot.get_target_object()
#         ObjectRecognition.predict(t_obj[0])
#     timer += 1


################################################
def save_image(o):
    from core.robot import get_robot_wrapper
    robot = get_robot_wrapper()
    camera = robot.camera
    file = f"./{o.getId()}.png"
    # camera.saveImage(file, 100)
    bytes = camera.getImage()
    img = Image.frombytes("RGBA", (camera.getWidth(), camera.getHeight()), bytes)
    b, g, r, a = img.split()
    img = Image.merge("RGBA", (r, g, b, a))

    pos, size = o.getPositionOnImage(), o.getSizeOnImage()
    left = pos[0] - 5
    top = pos[1] - 5
    right = left + size[0] + 5
    bottom = top + size[1] + 5
    cropped = img.crop((left, top, right, bottom))
    cropped.save(file)
    return


def predict(o):
    from keras.models import load_model
    from PIL import Image, ImageOps  # Install pillow instead of PIL
    import numpy as np
    save_image(o)
    # cropped.show()
    # Disable scientific notation for clarity
    np.set_printoptions(suppress=True)

    # Load the model
    model = load_model('core/keras_model.h5', compile=False)

    # Load the labels
    class_names = open('core/labels.txt', 'r').readlines()

    # Create the array of the right shape to feed into the keras model
    # The 'length' or number of images you can put into the array is
    # determined by the first position in the shape tuple, in this case 1.
    data = np.ndarray(shape=(1, 224, 224, 3), dtype=np.float32)

    # Replace this with the path to your image
    filename = str(o.getId()) + ".png"
    image = Image.open(filename).convert('RGB')

    # resize the image to a 224x224 with the same strategy as in TM2:
    # resizing the image to be at least 224x224 and then cropping from the center
    size = (224, 224)
    image = ImageOps.fit(image, size, Image.Resampling.LANCZOS)

    # turn the image into a numpy array
    image_array = np.asarray(image)

    # Normalize the image
    normalized_image_array = (image_array.astype(np.float32) / 127.0) - 1

    # Load the image into the array
    data[0] = normalized_image_array

    # run the inference
    prediction = model.predict(data)
    index = np.argmax(prediction)
    class_name = class_names[index]
    confidence_score = prediction[0][index]

    # print('Class:', class_name, end='')
    # print('Confidence score:', confidence_score)
    print("Object classified as: ", class_name)
    if os.path.exists(filename):
        os.remove(filename)
    if(class_name[0] != 0): #Yellow Goal object predicted
        return False
    else: #other object predicted
        return True
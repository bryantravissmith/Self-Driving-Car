import tensorflow as tf
import skimage.io
import skimage.transform
import numpy as np
import scipy.misc
from moviepy.editor import VideoFileClip

def load_graph(graph_file):
    """Loads a frozen inference graph"""
    graph = tf.Graph()
    with graph.as_default():
        od_graph_def = tf.GraphDef()
        with tf.gfile.GFile(graph_file, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')
    return graph

graph = load_graph('data/fitted_model/frozen_graph.pb')

logits = graph.get_tensor_by_name('logits:0')
image_input = graph.get_tensor_by_name('image_input:0')
keep_prob = graph.get_tensor_by_name('keep_prob:0')


num_classes = 2
image_shape = (160, 576)
img = scipy.misc.imresize(scipy.misc.imread('data/data_road/testing/image_2/um_000001.png'), image_shape)

with tf.Session(graph=graph) as sess:
    sess.run(tf.global_variables_initializer())

    def pipeline(image):
        #image_sp = scipy.misc.toimage(image)

        img = scipy.misc.imresize(image, image_shape)
        probs = sess.run(, {image_input: [img], keep_prob: 1.0})
        """
        probs = probs[:,1].reshape(image_shape[0], image_shape[1])
        segmentation = (probs).reshape(image_shape[0], image_shape[1], 1)
        mask = np.dot(segmentation, np.array([[0, 255, 0, 127]]))
        mask = scipy.misc.toimage(mask, mode="RGBA")
        street_im = scipy.misc.toimage(img)
        street_im.paste(mask, box=None, mask=mask)
        """
        return np.array(img)

    clip = VideoFileClip('driving.mp4', audio=False)
    new_clip = clip.fl_image(pipeline)

    # write to file
    new_clip.write_videofile('lane.mp4')

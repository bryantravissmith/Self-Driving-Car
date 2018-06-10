import os.path
import tensorflow as tf
import helper
import warnings
from distutils.version import LooseVersion
import project_tests as tests
from tqdm import tqdm


import skimage.io
import skimage.transform
import numpy as np
import scipy.misc
from moviepy.editor import VideoFileClip

# Check TensorFlow Version
assert LooseVersion(tf.__version__) >= LooseVersion('1.0'), 'Please use TensorFlow version 1.0 or newer.  You are using {}'.format(tf.__version__)
print('TensorFlow Version: {}'.format(tf.__version__))

# Check for a GPU
if not tf.test.gpu_device_name():
    warnings.warn('No GPU found. Please use a GPU to train your neural network.')
else:
    print('Default GPU Device: {}'.format(tf.test.gpu_device_name()))

LR = 0.0001
KP = 0.5

def load_vgg(sess, vgg_path):
    """
    Load Pretrained VGG Model into TensorFlow.
    :param sess: TensorFlow Session
    :param vgg_path: Path to vgg folder, containing "variables/" and "saved_model.pb"
    :return: Tuple of Tensors from VGG model (image_input, keep_prob, layer3_out, layer4_out, layer7_out)
    """
    # TODO: Implement function
    #   Use tf.saved_model.loader.load to load the model and weights
    vgg_tag = 'vgg16'
    vgg_input_tensor_name = 'image_input:0'
    vgg_keep_prob_tensor_name = 'keep_prob:0'
    vgg_layer3_out_tensor_name = 'layer3_out:0'
    vgg_layer4_out_tensor_name = 'layer4_out:0'
    vgg_layer7_out_tensor_name = 'layer7_out:0'

    tf.saved_model.loader.load(sess, [vgg_tag], vgg_path)
    graph = tf.get_default_graph()

    return (
        graph.get_tensor_by_name(vgg_input_tensor_name),
        graph.get_tensor_by_name(vgg_keep_prob_tensor_name),
        graph.get_tensor_by_name(vgg_layer3_out_tensor_name),
        graph.get_tensor_by_name(vgg_layer4_out_tensor_name),
        graph.get_tensor_by_name(vgg_layer7_out_tensor_name)
    )
tests.test_load_vgg(load_vgg, tf)


def layers(vgg_layer3_out, vgg_layer4_out, vgg_layer7_out, num_classes):
    """
    Create the layers for a fully convolutional network.  Build skip-layers using the vgg layers.
    :param vgg_layer3_out: TF Tensor for VGG Layer 3 output
    :param vgg_layer4_out: TF Tensor for VGG Layer 4 output
    :param vgg_layer7_out: TF Tensor for VGG Layer 7 output
    :param num_classes: Number of classes to classify
    :return: The Tensor for the last layer of output
    """
    # TODO: Implement function
    kernel_initializer = tf.truncated_normal_initializer(stddev = 0.01)

    conv_1x1_7 = tf.layers.conv2d(
        vgg_layer7_out, num_classes,
        1,
        strides=(1,1),
        padding='same',
        kernel_initializer= kernel_initializer,
        #kernel_regularizer=tf.contrib.layers.l2_regularizer(.001)
    )

    decode_1 = tf.layers.conv2d_transpose(
        conv_1x1_7,
        num_classes,
        4,
        strides=(2,2), padding='same',
        kernel_initializer= kernel_initializer,
        #kernel_regularizer=tf.contrib.layers.l2_regularizer(.001)
    )

    conv_1x1_4 = tf.layers.conv2d(
        vgg_layer4_out,
        num_classes,
        1,
        strides=(1,1),
        padding='same',
        kernel_initializer= kernel_initializer,
        #kernel_regularizer=tf.contrib.layers.l2_regularizer(.001)
    )

    decode_1_skip4 = tf.add(decode_1, conv_1x1_4)

    decode_2 = tf.layers.conv2d_transpose(
        decode_1_skip4,
        num_classes,
        4,
        strides=(2,2), padding='same',
        kernel_initializer= kernel_initializer,
        #kernel_regularizer=tf.contrib.layers.l2_regularizer(.001)
    )

    conv_1x1_3 = tf.layers.conv2d(
        vgg_layer3_out,
        num_classes,
        1,
        strides=(1,1),
        padding='same',
        kernel_initializer= kernel_initializer,
        #kernel_regularizer=tf.contrib.layers.l2_regularizer(.001)
    )

    decode_2_skip3 = tf.add(decode_2, conv_1x1_3)

    decode_3 = tf.layers.conv2d_transpose(
        decode_2_skip3,
        num_classes,
        16,
        strides=(8,8),
        padding='same',
        kernel_initializer= kernel_initializer,
        #kernel_regularizer=tf.contrib.layers.l2_regularizer(.001)
    )

    return decode_3

tests.test_layers(layers)


def optimize(nn_last_layer, correct_label, learning_rate, num_classes):
    """
    Build the TensorFLow loss and optimizer operations.
    :param nn_last_layer: TF Tensor of the last layer in the neural network
    :param correct_label: TF Placeholder for the correct label image
    :param learning_rate: TF Placeholder for the learning rate
    :param num_classes: Number of classes to classify
    :return: Tuple of (logits, train_op, cross_entropy_loss)
    """
    # TODO: Implement function
    logits = tf.reshape(nn_last_layer, (-1, num_classes), name='logits')
    labels = tf.reshape(correct_label, (-1, num_classes))
    cross_entropy_loss = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits_v2(logits=logits, labels=labels))
    op = tf.train.AdamOptimizer(learning_rate)
    train_op = op.minimize(cross_entropy_loss)

    return logits, train_op, cross_entropy_loss
tests.test_optimize(optimize)


def train_nn(sess, epochs, batch_size, get_batches_fn, train_op, cross_entropy_loss, input_image,
             correct_label, keep_prob, learning_rate):
    """
    Train neural network and print out the loss during training.
    :param sess: TF Session
    :param epochs: Number of epochs
    :param batch_size: Batch size
    :param get_batches_fn: Function to get batches of training data.  Call using get_batches_fn(batch_size)
    :param train_op: TF Operation to train the neural network
    :param cross_entropy_loss: TF Tensor for the amount of loss
    :param input_image: TF Placeholder for input images
    :param correct_label: TF Placeholder for label images
    :param keep_prob: TF Placeholder for dropout keep probability
    :param learning_rate: TF Placeholder for learning rate
    """
    # TODO: Implement function
    sess.run(tf.global_variables_initializer())

    pbar = tqdm(range(epochs))
    for i in pbar:
        for images, labels in get_batches_fn(batch_size):
            if i <= epochs / 2:
                _, loss = sess.run([
                    train_op, cross_entropy_loss,
                ], {
                    input_image: images,
                    correct_label: labels,
                    learning_rate: LR,
                    keep_prob: KP
                })
            else:
                _, loss = sess.run([
                    train_op, cross_entropy_loss,
                ], {
                    input_image: images,
                    correct_label: labels,
                    learning_rate: LR,
                    keep_prob: KP/100.0
                })
            pbar.set_description("Loss: {}".format(loss))

#tests.test_train_nn(train_nn)


def run():
    num_classes = 2
    image_shape = (160, 576)
    data_dir = './data'
    runs_dir = './runs'
    tests.test_for_kitti_dataset(data_dir)

    # Download pretrained vgg model
    helper.maybe_download_pretrained_vgg(data_dir)

    # OPTIONAL: Train and Inference on the cityscapes dataset instead of the Kitti dataset.
    # You'll need a GPU with at least 10 teraFLOPS to train on.
    #  https://www.cityscapes-dataset.com/

    with tf.Session() as sess:
        # Path to vgg model
        vgg_path = os.path.join(data_dir, 'vgg')
        # Create function to get batches
        get_batches_fn = helper.gen_batch_function(os.path.join(data_dir, 'data_road/training'), image_shape)

        # OPTIONAL: Augment Images for better results
        #  https://datascience.stackexchange.com/questions/5224/how-to-prepare-augment-images-for-neural-network

        # TODO: Build NN using load_vgg, layers, and optimize function
        input_image, keep_prob, layer3, layer4, layer7 = load_vgg(sess, vgg_path)
        output = layers(layer3, layer4, layer7, num_classes)


        correct_label = tf.placeholder(tf.float32, [None, image_shape[0], image_shape[1], num_classes], 'correct_labels')
        learning_rate = tf.placeholder(tf.float32, name='learning_rate')

        logits, train_op, cross_entropy_loss = optimize(output, correct_label, learning_rate, num_classes)
        probs = tf.nn.softmax(logits)
        # TODO: Train NN using the train_nn function
        train_nn(sess, 20, 8, get_batches_fn, train_op, cross_entropy_loss, input_image, correct_label, keep_prob, learning_rate)
        # TODO: Save inference data using helper.save_inference_samples

        helper.save_inference_samples(runs_dir, data_dir, sess, image_shape, logits, keep_prob, input_image)

        tf.train.write_graph(
            tf.get_default_graph(),
            './data/fitted_model/',
            'fitted_model.pb',
            as_text=False
        )
        saver = tf.train.Saver()
        saver.save(sess, './data/fitted_model/fitted_model.ckpt')
        # OPTIONAL: Apply the trained model to a video

        def pipeline(image):
            #image_sp = scipy.misc.toimage(image)

            img = scipy.misc.imresize(image, image_shape)
            p = sess.run(probs, {input_image: [img], keep_prob: 1.0})
            p = p[:,1].reshape(image_shape[0], image_shape[1])
            segmentation = (p).reshape(image_shape[0], image_shape[1], 1)
            mask = np.dot(segmentation, np.array([[0, 255, 0, 127]]))
            mask = scipy.misc.toimage(mask, mode="RGBA")
            street_im = scipy.misc.toimage(img)
            street_im.paste(mask, box=None, mask=mask)
            return np.array(street_im)

        clip = VideoFileClip('project_video.mp4', audio=False)
        new_clip = clip.fl_image(pipeline)

        # write to file
        new_clip.write_videofile('lane.mp4')

if __name__ == '__main__':
    run()

import glob
import cv2
import numpy as np
from skimage.feature import hog
from scipy.ndimage.measurements import label
from collections import deque

def load_training_data_rgb(base_dir):
    non_vehicles = []
    vehicles = []

    for img_loc in glob.glob('{}/non-vehicles/GTI/*.png'.format(base_dir)):
        non_vehicles.append(cv2.cvtColor(cv2.imread(img_loc), cv2.COLOR_BGR2RGB))

    for img_loc in glob.glob('{}/non-vehicles/Extras/*.png'.format(base_dir)):
        non_vehicles.append(cv2.cvtColor(cv2.imread(img_loc), cv2.COLOR_BGR2RGB))

    for img_loc in glob.glob('{}/vehicles/GTI_Left/*.png'.format(base_dir)):
        vehicles.append(cv2.cvtColor(cv2.imread(img_loc), cv2.COLOR_BGR2RGB))

    for img_loc in glob.glob('{}/vehicles/GTI_Right/*.png'.format(base_dir)):
        vehicles.append(cv2.cvtColor(cv2.imread(img_loc), cv2.COLOR_BGR2RGB))

    for img_loc in glob.glob('{}/vehicles/GTI_MiddleClose/*.png'.format(base_dir)):
        vehicles.append(cv2.cvtColor(cv2.imread(img_loc), cv2.COLOR_BGR2RGB))

    for img_loc in glob.glob('{}/vehicles/GTI_Far/*.png'.format(base_dir)):
        vehicles.append(cv2.cvtColor(cv2.imread(img_loc), cv2.COLOR_BGR2RGB))

    for img_loc in glob.glob('{}/vehicles/KITTI_extracted/*.png'.format(base_dir)):
        vehicles.append(cv2.cvtColor(cv2.imread(img_loc), cv2.COLOR_BGR2RGB))

    return vehicles, non_vehicles


def color_normalization(img):
    imgc = img.copy().astype(np.float32) / img.max()
    color_norm = np.dstack([imgc.sum(axis=2)]*3).astype(np.float32)
    imgc[color_norm != 0] = imgc[color_norm != 0] / color_norm[color_norm != 0]
    return imgc / imgc.max()

def intensity_normalization(img):
    img_copy = img.copy().astype(np.float32) / img.max()
    intensity_norm = 3 * img_copy.sum(axis=0).sum(axis=0) / img.shape[0] / img.shape[1]
    result = img_copy / (intensity_norm+1e-6)
    return result / result.max()

def img_norm(img, ksize=5):
    kernel = np.ones((ksize,ksize),np.float32)/ksize**2
    return np.uint8(255*intensity_normalization(
            color_normalization(cv2.filter2D(img,-1,kernel))
    ))


def extract_hog(img_ch, pix_per_cell = 8, cell_per_block = 2, orient = 9, vis=True, vector=False ):
    return hog(img_ch, orientations=orient,
              pixels_per_cell=(pix_per_cell, pix_per_cell),
              cells_per_block=(cell_per_block, cell_per_block),
              visualise=vis, feature_vector=vector, block_norm='L2-Hys')

def hog_feature_vector(img):
    return np.concatenate([
        extract_hog(img[:,:,i], vis=False, vector=True)
        for i in range(img.shape[2])
    ])


def spacial_features(img, size=(32, 32)):
    return cv2.resize(img, size).ravel()

def color_features(img, nbins=32, bins_range=(0, 256)):
    return np.concatenate([
        np.histogram(img[:,:,i], bins=nbins, range=bins_range)[0]
        for i in range(3)
    ])

def get_spacial_color_features(img, size=(32,32), nbins=32, bins_range=(0,256)):
    return np.concatenate([
        spacial_features(img, size=size),
        color_features(img, nbins=nbins, bins_range=bins_range)
    ])


def create_car_heatmap(img, pipe, scales=[1], ystart=400, ystop=700, orient=9, pix_per_cell=8, cell_per_block=2, belief=None):

    heatmap = np.zeros_like(img[:,:,0]).astype(np.float32)

    for scale in scales:
        ctrans_tosearch = img.copy()[ystart:ystop,:,:]
        if scale != 1:
            imshape = ctrans_tosearch.shape
            ctrans_tosearch = cv2.resize(ctrans_tosearch, (np.int(imshape[1]/scale), np.int(imshape[0]/scale)))

        ycrcb = cv2.cvtColor(ctrans_tosearch, cv2.COLOR_RGB2YCrCb)

        ch1 = ycrcb[:,:,0]
        ch2 = ycrcb[:,:,1]
        ch3 = ycrcb[:,:,2]

        nxblocks = (ch1.shape[1] // pix_per_cell) - cell_per_block + 1
        nyblocks = (ch1.shape[0] // pix_per_cell) - cell_per_block + 1
        nfeat_per_block = orient*cell_per_block**2

        # 64 was the orginal sampling rate, with 8 cells and 8 pix per cell
        window = 64

        nblocks_per_window = (window // pix_per_cell) - cell_per_block + 1

        cells_per_step = 2 # Instead of overlap, define how many cells to step

        nxsteps = int((nxblocks - nblocks_per_window) // cells_per_step + 1)
        nysteps = int((nyblocks - nblocks_per_window) // cells_per_step + 1)

        # Compute individual channel HOG features for the entire image
        hog1 = extract_hog(ch1, pix_per_cell, cell_per_block,orient, vis=False, vector=False)
        hog2 = extract_hog(ch2, pix_per_cell, cell_per_block,orient, vis=False, vector=False)
        hog3 = extract_hog(ch3, pix_per_cell, cell_per_block,orient, vis=False, vector=False)

        for xb in range(nxsteps):
            for yb in range(nysteps):
                ypos = yb*cells_per_step
                xpos = xb*cells_per_step
                # Extract HOG for this patch
                hog_feat1 = hog1[ypos:ypos+nblocks_per_window, xpos:xpos+nblocks_per_window].ravel()
                hog_feat2 = hog2[ypos:ypos+nblocks_per_window, xpos:xpos+nblocks_per_window].ravel()
                hog_feat3 = hog3[ypos:ypos+nblocks_per_window, xpos:xpos+nblocks_per_window].ravel()
                hog_features = np.hstack((hog_feat1, hog_feat2, hog_feat3))

                xleft = xpos*pix_per_cell
                ytop = ypos*pix_per_cell

                subimg = cv2.resize(ctrans_tosearch[ytop:ytop+window, xleft:xleft+window], (64,64))

                features = np.concatenate([
                    get_spacial_color_features(subimg),
                    hog_features
                ])

                pred = pipe.predict([features])

                xbox_left = np.int(xleft*scale)
                ytop_draw = np.int(ytop*scale)
                win_draw = np.int(window*scale)

                heatmap[
                    ytop_draw+ystart:ytop_draw+win_draw+ystart,
                    xbox_left:xbox_left+win_draw,
                ] += pred

    return heatmap


def draw_labeled_bboxes(img, heatmap):
    # Iterate through all detected cars
    labels = label(heatmap)
    img_copy = img.copy()
    for car_number in range(1, labels[1]+1):
        # Find pixels with each car_number label value
        nonzero = (labels[0] == car_number).nonzero()
        # Identify x and y values of those pixels
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        # Define a bounding box based on min/max x and y
        bbox = ((np.min(nonzerox), np.min(nonzeroy)), (np.max(nonzerox), np.max(nonzeroy)))
        # Draw the box on the image
        cv2.rectangle(img_copy, bbox[0], bbox[1], (0,0,255), 6)
    # Return the image

    return img_copy


class CarDetectionPipeline():

    def __init__(self, model):
        self.buffer = deque(maxlen = 15)
        self.model = model

    def transform(self, img):
        base = np.zeros((img.shape[0],img.shape[1]*2, img.shape[2]))


        car_heatmap = create_car_heatmap(img, self.model, scales=[1.0], ystart=375, ystop=375+64*2)
        car_heatmap += create_car_heatmap(img, self.model, scales=[1.33], ystart=400, ystop=700)
        car_heatmap[car_heatmap <= 3] = 0
        boxes = (label(car_heatmap > 0)[0] > 0).astype(np.int)
        self.buffer.append(boxes)

        tmp = (np.array(self.buffer).sum(axis=0) == len(self.buffer)).astype(int)*255

        if len(self.buffer) > 3:
            img = draw_labeled_bboxes(img, tmp)

        base[:,:img.shape[1]:,:] = img
        base[:,img.shape[1]:,:] = np.dstack([np.uint8(255 * car_heatmap / car_heatmap.max())]*3)
        #base[:,img.shape[1]:,:] = np.dstack([np.uint8(255 * boxes)]*3)

        return np.uint8(base)

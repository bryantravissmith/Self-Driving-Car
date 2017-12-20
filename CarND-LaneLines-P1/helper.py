import math
import numpy as np
import cv2

rho = 1
theta = np.pi/180
threshold = 1
min_line_len = 1
max_line_gap = 1

upper = 0.6
#Right Region of Interest
def get_vertices(image):
    right_vertices = np.array([[
        (image.shape[1]*.5,image.shape[0]*upper),
        (image.shape[1]*.5,image.shape[0]),
        (image.shape[1]*.95, image.shape[0]), 
        (image.shape[1]*.55,image.shape[0]*upper)
    ]], dtype=np.int32)

    #Left Region of Interest
    left_vertices = np.array([[
        (image.shape[1]*.45,image.shape[0]*upper),
        (image.shape[1]*.05,image.shape[0]),
        (image.shape[1]*.5,image.shape[0]), 
         (image.shape[1]*.5,image.shape[0]*upper),
    ]], dtype=np.int32)
    
    return left_vertices, right_vertices  
    
def grayscale(img):
    """Applies the Grayscale transform
    This will return an image with only one color channel
    but NOTE: to see the returned image as grayscale
    (assuming your grayscaled image is called 'gray')
    you should call plt.imshow(gray, cmap='gray')"""
    return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    # Or use BGR2GRAY if you read an image with cv2.imread()
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
def canny(img, low_threshold, high_threshold):
    """Applies the Canny transform"""
    return cv2.Canny(img, low_threshold, high_threshold)

def gaussian_blur(img, kernel_size):
    """Applies a Gaussian Noise kernel"""
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

def region_of_interest(img, vertices):
    """
    Applies an image mask.
    
    Only keeps the region of the image defined by the polygon
    formed from `vertices`. The rest of the image is set to black.
    """
    #defining a blank mask to start with
    mask = np.zeros_like(img)   
    
    #defining a 3 channel or 1 channel color to fill the mask with depending on the input image
    if len(img.shape) > 2:
        channel_count = img.shape[2]  # i.e. 3 or 4 depending on your image
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255
        
    #filling pixels inside the polygon defined by "vertices" with the fill color    
    cv2.fillPoly(mask, vertices, ignore_mask_color)
    
    #returning the image only where mask pixels are nonzero
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image

def draw_lines(img, lines, color=[255, 0, 0], thickness=4):
    """
    NOTE: this is the function you might want to use as a starting point once you want to 
    average/extrapolate the line segments you detect to map out the full
    extent of the lane (going from the result shown in raw-lines-example.mp4
    to that shown in P1_example.mp4).  
    
    Think about things like separating line segments by their 
    slope ((y2-y1)/(x2-x1)) to decide which segments are part of the left
    line vs. the right line.  Then, you can average the position of each of 
    the lines and extrapolate to the top and bottom of the lane.
    
    This function draws `lines` with `color` and `thickness`.    
    Lines are drawn on the image inplace (mutates the image).
    If you want to make the lines semi-transparent, think about combining
    this function with the weighted_img() function below
    """

    if lines is None:
        return img
    
    # Collect the Positions for the line segments to perform linear regression on all points
    xs = []
    ys = []
    slopes = []
    for line in lines:
        for x1,y1,x2,y2 in line:
            slope = (y2-y1)/(x2-x1)
            slopes.append(slope)
            xs.extend([x1,x2])
            ys.extend([y1,y2])
            
    xs = np.array(xs)
    ys = np.array(ys)
    
    # Fit a line to the line segment points
    fit = np.poly1d( np.polyfit(xs, ys, 1))
    resids = np.abs(ys - fit(xs))
    
    #Refit on the points closest to line for stability 
    fit = np.poly1d(np.polyfit(xs[resids <= np.percentile(resids,80)], ys[resids <= np.percentile(resids,80)], 1))
    cv2.line(img, (0, int(fit(0))), (img.shape[1], int(fit(img.shape[1]))), color, thickness)
            
def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap):
    """
    `img` should be the output of a Canny transform.
        
    Returns an image with hough lines drawn.
    """
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
    line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    draw_lines(line_img, lines)
    return line_img

# Python 3 has support for cool math symbols.

def weighted_img(img, initial_img, α=0.8, β=1.0, λ=0.):
    """
    `img` is the output of the hough_lines(), An image with lines drawn on it.
    Should be a blank image (all black) with lines drawn on it.
    
    `initial_img` should be the image before any processing.
    
    The result image is computed as follows:
    
    initial_img * α + img * β + λ
    NOTE: initial_img and img must be the same shape!
    """
    return cv2.addWeighted(initial_img, α, img, β, λ)

def image_pipeline(image):
    rho = 1
    theta = np.pi/180
    threshold = 1
    min_line_len = 1
    max_line_gap = 1
    
    upper = 0.6
    #Right Region of Interest
    right_vertices = np.array([[
        (image.shape[1]*.5,image.shape[0]*upper),
        (image.shape[1]*.5,image.shape[0]),
        (image.shape[1]*.95, image.shape[0]), 
        (image.shape[1]*.55,image.shape[0]*upper)
    ]], dtype=np.int32)

    #Left Region of Interest
    left_vertices = np.array([[
        (image.shape[1]*.45,image.shape[0]*upper),
        (image.shape[1]*.05,image.shape[0]),
        (image.shape[1]*.5,image.shape[0]), 
         (image.shape[1]*.5,image.shape[0]*upper),
    ]], dtype=np.int32)

    #convert to hsv to try to remove effects from shadows and lighting
    img_hsv = cv2.cvtColor(255-image, cv2.COLOR_RGB2HSV)
    ## same value
    img_hsv[:,:,2] = 255
    ## same hue
    img_hsv[:,:,0] = 0
    ## convert back to RGB and gray scale
    img_gray = gaussian_blur(grayscale(cv2.cvtColor(img_hsv, cv2.COLOR_HSV2RGB)),5)
    ## edges
    
    img_edges = canny(img_gray,40,120)
    img_edges = region_of_interest(img_edges, left_vertices) + region_of_interest(img_edges, right_vertices)
    
    img_left_edges = region_of_interest(img_edges, left_vertices)
    img_left_line = region_of_interest(hough_lines(img_left_edges, rho, theta, threshold, min_line_len, max_line_gap),left_vertices)
    
    img_right_edges = region_of_interest(img_edges, right_vertices)
    img_right_line = region_of_interest(hough_lines(img_right_edges, rho, theta, threshold, min_line_len, max_line_gap),right_vertices)
    
    img_line = img_right_line + img_left_line
    
    return weighted_img(img_line, image) #np.dstack([img_edges,img_edges,img_edges])

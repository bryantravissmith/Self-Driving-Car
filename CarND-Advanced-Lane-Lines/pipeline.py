from glob import glob
import cv2
import numpy as np
from sklearn.preprocessing import PolynomialFeatures

def calibration():
    nx = 9
    ny = 6

    objp = np.zeros((ny*nx,3), np.float32)
    objp[:,:2] = np.mgrid[0:ny, 0:nx].T.reshape(-1,2)

    imgpoints, objpoints = [], []

    calibration_imgs = []
    for img_loc in glob('camera_cal/calibration*.jpg'):
        calibration_imgs.append(cv2.cvtColor(cv2.imread(img_loc), cv2.COLOR_BGR2RGB))
        gray = cv2.cvtColor(cv2.imread(img_loc), cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (ny, nx),None)
        if ret == True:
            objpoints.append(objp)
            imgpoints.append(corners)

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, calibration_imgs[0].shape[:2][::-1],None,None)
    return ret, mtx, dist, rvecs, tvecs

def white_select(img, min_thresh=200):
    yuv = cv2.cvtColor(img, cv2.COLOR_RGB2YUV)
    blank = np.zeros_like(yuv[:,:,0])
    blank[(yuv[:,:,0] >= min_thresh)] = 1
    return blank

def yellow_select(img, min_thresh = 140):
    yuv = cv2.cvtColor(img, cv2.COLOR_RGB2YUV)
    blank = np.zeros_like(yuv[:,:,1])
    blank[(yuv[:,:,1] >= min_thresh)] = 1
    return blank

def yuv_select(img, y_thresh=200, u_thresh=150):
    y = yellow_select(img, min_thresh=u_thresh)
    u = white_select(img, min_thresh=y_thresh)
    blank = np.zeros_like(img[:,:,0])
    blank[(u>0)|(y>0)] = 1
    return blank


def get_vanishing_point(img):
    rho = 1
    theta = np.pi/180
    threshold = 1
    min_line_len = 50
    max_line_gap = 5
    k_size = 5
    kernel = np.ones((k_size,k_size),np.float32)/k_size**2

    median = cv2.filter2D(cv2.cvtColor(img, cv2.COLOR_RGB2GRAY),-1,kernel)
    edge = cv2.Canny(median, 30,160)
    lines = cv2.HoughLinesP(edge, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)

    blank = np.zeros_like(img)[:,:,0]
    x = np.arange(0,blank.shape[1]).astype(np.int)
    for line in lines:
        slope = (line[0,3]-line[0,1])/(line[0,2]-line[0,0])
        intercept = (line[0,3] - slope*line[0,2])
        if slope != 0 and np.isfinite(intercept):
            y = (slope*x+intercept).astype(np.int)
            pts = np.vstack([y,x]).T
            pts = pts[(y>=0)&(y<=blank.shape[0]-1),:]
            blank[pts[:,0],pts[:,1]] += 1

    ys = blank.sum(axis=1)
    i_ys = (ys > 0.95*ys.max()).nonzero()[0]
    xs = blank[i_ys].sum(axis=0)

    y_vanish = np.median(i_ys).astype(int)
    x_vanish = np.median((xs > 0.95*xs.max()).nonzero()[0]).astype(int)
    return (x_vanish, y_vanish)


def get_matrix(img, v_y, dy=20, correction=15):
    dst =  np.float32([
            [0, 0],
            [0, img.shape[0]],
            [img.shape[1], img.shape[0]],
            [img.shape[1], 0]
        ])

    slope = (img.shape[0] - v_y) * 2.0 / img.shape[1]

    src = np.float32([
            [img.shape[1]//2 - dy/slope - correction, v_y+dy],
            [-dy/slope, img.shape[0]+dy],
            [img.shape[1]+dy/slope, img.shape[0]+dy],
            [img.shape[1]//2  + dy/slope + correction, v_y+dy]
        ])

    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst, src)
    return M, Minv



class LaneDetector():

    def __init__(self, l2 = 1e8, ym_per_pix=30/720, xm_per_pix=3.7/700, eps=0.8):
        self.polytransform = PolynomialFeatures(2, include_bias=True)
        self.coef = None
        self.L = np.zeros((6,6))
        self.L[1,1] = 1
        self.L[1,4] = -1
        self.L[2,2] = 2
        self.L[2,5] = -1
        self.L[4,1] = -1
        self.L[4,4] = 1
        self.L[5,2] = -1
        self.L[5,5] = 2
        self.L = self.L*l2
        self.ym_per_pix = ym_per_pix
        self.xm_per_pix = xm_per_pix
        self.eps = eps
        self.prev_X = None
        self.prev_Y = None
        self.r = 0
        self.offset = 0


    def R(self):
        return np.round(self.r/1000,1)

    def Offset(self):
        return np.round(self.offset,2)

    def gen_data(self, img):
        y, x = img.nonzero()
        y1 = y[x < img.shape[1]//2] * self.ym_per_pix
        x1 = x[x < img.shape[1]//2] * self.xm_per_pix

        y2 = y[x >= img.shape[1]//2] * self.ym_per_pix
        x2 = x[x >= img.shape[1]//2] * self.xm_per_pix

        try:
            y1p = self.polytransform.fit_transform(y1.reshape((y1.size,1)))
            y2p = self.polytransform.fit_transform(y2.reshape((y2.size,1)))

            Y = np.vstack([
                np.hstack([y1p, np.zeros_like(y1p)]),
                np.hstack([np.zeros_like(y2p), y2p])
            ])
            X = np.concatenate([x1, x2])
            self.prev_X = X
            self.prev_Y = Y
            return X, Y
        except:
            return self.prev_X, self.prev_Y

    def update(self, img):

        X, Y = self.gen_data(img)
        if X is not None:
            inv = np.linalg.inv(Y.T.dot(Y) + self.L)
            if self.coef is None:
                self.coef = inv.dot(Y.T.dot(X))
            else:
                self.coef = inv.dot(Y.T.dot(X)) * self.eps + (1 - self.eps)*self.coef

            r1 = 0.5* np.power(1 + np.power(2*self.coef[2]* (img[0]*self.ym_per_pix) + self.coef[1], 2), 1.5)/np.abs(self.coef[2])
            r2 = 0.5* np.power(1 + np.power(2*self.coef[5]* (img[0]*self.ym_per_pix) + self.coef[4], 2), 1.5)/np.abs(self.coef[5])
            if self.r == 0:
                self.r = np.mean(r1+r2)
            else:
                self.r = np.mean(r1+r2)*self.eps / 2 + (1-self.eps)*self.r

            val = img.shape[0]*self.ym_per_pix
            offset = (
                np.array(
                    [[1,val, val**2,0,0,0],
                    [0,0,0,1,val,val**2]]
                ).dot(
                    self.coef
                ).mean() / self.xm_per_pix - img.shape[1]//2
            ) * self.xm_per_pix

            if self.offset==0:
                self.offset = offset
            else:
                self.offset = offset * self.eps + (1-self.eps)*self.offset


    def draw(self, img_2d):


        if self.coef is None:
            return np.dstack([img_2d]*3)
        f1 = np.poly1d(self.coef[:3][::-1])
        f2 = np.poly1d(self.coef[3:][::-1])
        ys = np.linspace(0,img_2d.shape[1],2000) * self.ym_per_pix
        pts1 = np.array(
            list(zip(
                f1(ys)/self.xm_per_pix,
                ys/self.ym_per_pix)),
            np.int32
        ).reshape((-1,1,2))
        pts2 = np.array(
            list(zip(
                f2(ys)/self.xm_per_pix,
                ys/self.ym_per_pix
            )),
            np.int32
        ).reshape((-1,1,2))

        img_copy = np.dstack([img_2d]*3)
        cv2.fillPoly(img_copy, np.hstack([pts2,pts1]), (0,255,0))
        cv2.polylines(img_copy, pts1, True,(255,0,0),25)
        cv2.polylines(img_copy, pts2, True,(255,0,0),25)
        return img_copy


class LanePipeLine():

    def __init__(self, mtx, dist, return_warped = False):
        self.mtx = mtx
        self.dist = dist
        self.initalized = False
        self.midpoint = None
        self.M = None
        self.Minv = None
        self.vanishing_pnt = None
        self.lane_detector = LaneDetector(l2 = 1e6, eps=0.5)
        self.return_warped = return_warped

    def undistort_image(self, img):
        return cv2.undistort(img, self.mtx, self.dist, None, self.mtx)


    def transform(self, img):

        undist = self.undistort_image(img)
        binary = yuv_select(undist)

        if not self.initalized:
            self.vanishing_pnt = get_vanishing_point(undist)
            self.M, self.Minv = get_matrix(img, self.vanishing_pnt[1], dy=40)
            self.initalized = True

        warped = cv2.warpPerspective(
            binary,
            self.M,
            img.shape[:2][::-1]
        )

        #return (np.dstack([warped, warped, warped]) > 0).astype(np.uint8)*255

        self.lane_detector.update(warped)
        overlay = self.lane_detector.draw(warped)

        if self.return_warped:
            return cv2.addWeighted((np.dstack([warped, warped, warped]) > 0).astype(np.uint8)*255, 1, overlay, 0.3, 0)

        overlay = cv2.warpPerspective(
            overlay,
            self.Minv,
            overlay.shape[:2][::-1]
        )


        result = cv2.addWeighted(undist, 1, overlay, 0.3, 0)
        font = cv2.FONT_HERSHEY_SIMPLEX
        result = cv2.putText(result,'R = {}km, Offset = {}m '.format(
            self.lane_detector.R(),
            self.lane_detector.Offset()
        ),(50,50), font, 1,(255,255,255),2,cv2.LINE_AA)
        return result


if __name__ == "__main__":

    ret, mtx, dist, rvecs, tvecs = calibration()

    def undistort_image(img):
        return cv2.undistort(img, mtx, dist, None, mtx)

import numpy as np

# offsets of each variable in the state vector
iX = 0
iY = 1
iV = 2
iHX = 3
iHY = 4
NUMVARS = iHY + 1


class KF:
    def __init__(self, initial_x: float,
                       initial_y: float,
                       #initial_z: float,
                       initial_v: float,
                       initial_h_x: float,
                       initial_h_y: float,
                       accel_variance: float) -> None:
        # mean of state GRV
        self._x = np.zeros(NUMVARS)

        self._x[iX] = initial_x
        self._x[iV] = initial_v
        self._x[iY] = initial_y
        #self._x[iZ] = initial_z
        self._x[iHX] = initial_h_x
        self._x[iHY] = initial_h_y
        self.omega = 0.1
        self._accel_variance = accel_variance
        self.R90=np.array([[0.0,-1.0],[1.0,0.0]])
        self.beacon_id = [42867, 42928,  42929,  44530,  44531,  44532,  44533,  44534,  44535,  44536,  44537,  44538,  44540]
        self.beacon =    np.array([[11700, 16244,  7824,   2000,   21369,  26163,  26163,  31000,  35766,  35766,  40205,  40204,  16560],[5999,  10150,  5726,   4499,   6534,   9939,   3699,   6519,   10012,  3522,   11684,  4363,   3549]])

        # covariance of state GRV
        self._P = np.eye(NUMVARS)#( [1,0,0,0,0],[0,1,0,0,0],[0,0,1,0,0],[0,0,0,1,0],[0,0,0,0,1])

        # have to update the rest of the code from this point un:
    def predict(self, dt: float) -> None:
        # x = F x
        # P = F P Ft + G Gt a
        #F = np.eye(NUMVARS)
        print(1.0+self.R90.dot(dt).dot(self.omega))
        F = np.array([  [1.0, 0.0,  self._x[iHX]*dt,            self._x[iV]*dt,             0.0],
                        [0.0, 1.0,  self._x[iHY]*dt,            0.0,                          self._x[iV]*dt],
                        [0.0, 0.0,  1.0,                          0.0,                          0.0],
                        [0.0, 0.0,  1.0+self.R90.dot(dt).dot(self.omega),   0.0,                          0.0],
                        [0.0, 0.0,  0.0,                          1.0+self.R90.dot(dt).dot(self.omega),   0.0]])
        #F[iX, iV] = dt
        #F[iX, iY, iZ, iV, iHX, iHY] = dt
        new_x = F.dot(self._x)

        G = np.array([  [0.5 * dt**2],
                        [0],
                        [self._x[iV]],
                        [0],
                        [0]])
        new_P = F.dot(self._P).dot(F.T) + G.dot(G.T) * self._accel_variance

        self._P = new_P
        self._x = new_x

    def update(self, id: int,  meas_value: float, meas_variance: float):
        index_of_data = self.beacon_id.index(id)
        # y = z - H x
        # S = H P Ht + R
        # K = P Ht S^-1
        # x = x + K y
        # P = (I - K H) * P

        H = np.array([  [np.abs(self.beacon[0,index_of_data]-self._x[iHX])*np.sign(self.beacon[0,index_of_data]-self._x[iHX])*-2],
                        [np.abs(self.beacon[1,index_of_data]-self._x[iHY])*np.sign(self.beacon[1,index_of_data]-self._x[iHY])*-2],
                        [0],
                        [0],
                        [0]])

        z = np.array([meas_value])
        R = np.array([meas_variance])
        print(self._x)
        print(len(H),len(H[0]))

        y = z - H.dot(self._x)
        S = H.dot(self._P).dot(H.T) + R

        K = self._P.dot(H.T).dot(np.linalg.inv(S))

        new_x = self._x + K.dot(y)
        new_P = (np.eye(2) - K.dot(H)).dot(self._P)

        self._P = new_P
        self._x = new_x

    @property
    def cov(self) -> np.array:
        return self._P

    @property
    def mean(self) -> np.array:
        return self._x

    @property
    def pos(self) -> float:
        return self._x[iX]

    @property
    def vel(self) -> float:
        return self._x[iV]

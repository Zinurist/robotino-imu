#!/usr/bin/env python
import math
#values for sm and ml curves
btw_sm = math.pi*(3/8)
btw_ml = math.pi*(3/16)


"""
Returns the labels and a function for the given mv_type for recording. The labels represent a list of labels associated with mvtype. The function can be called to record a single sample, it returns the sample and extra info, see movement.py.
Possible mvtypes are:
-"angle" with degree+left
-"curve_s/m/l/sm/ml", "object" with left
-"straight", "wall", "foot"
-None with angle+left for custom curve

@param mvs: the movement set, an instance that contains all function for recording, see movement.py
@param mvtype: the type of sample to record
@param angle: angular velocity to use then recording a custom curve sample (ignored else)
@param degree: degree to use for an angle sample (ignored else)
@param left: left or right for curves or angles
@return the labels of the mvtype and the recording function
"""
def get_mv(mvs, mvtype=None, angle=None, degree=None, left=True):
    dir_txt = '_l' if left else '_r'

    if angle is not None and mvtype is None:
        if angle > btw_sm:
            labels=['curve_s', 'custom', left, 'curve_s' + dir_txt]
        elif radius > btw_ml:
            labels=['curve_m', 'custom', left, 'curve_m' + dir_txt]
        else:
            labels=['curve_l', 'custom', left, 'curve_l' + dir_txt]
        def fn(): 
            return mvs.mv_curve(velocity=1, angle=angle, left=left)
        return labels,fn

    if mvtype == 'curve_s':
        def fn(): return mvs.mv_curve_small(left)
        return [mvtype, left, mvtype + dir_txt],fn
    if mvtype == 'curve_m':
        def fn(): return mvs.mv_curve_middle(left)
        return [mvtype, left, mvtype + dir_txt],fn
    if mvtype == 'curve_l':
        def fn(): return mvs.mv_curve_large(left)
        return [mvtype, left, mvtype + dir_txt],fn
    if mvtype == 'curve_sm':
        def fn(): return mvs.mv_curve_sm(left)
        return [mvtype, left, mvtype + dir_txt],fn
    if mvtype == 'curve_ml':
        def fn(): return mvs.mv_curve_ml(left)
        return [mvtype, left, mvtype + dir_txt],fn

    if mvtype == 'object':
        def fn(): return mvs.mv_straight(4)
        return [mvtype, mvtype+dir_txt],fn
    if mvtype == 'straight':
        return [mvtype],mvs.mv_straight
    if mvtype == 'wall' or mvtype == 'foot':
        return [mvtype],mvs.mv_wall

    if degree is not None and mvtype == 'angle':
        def fn(): return mvs.mv_angle(degree,left)
        return [mvtype, mvtype + '_' + str(degree), mvtype + '_' + str(degree) + dir_txt],fn
    
    raise ValueError('Unknown movement type')

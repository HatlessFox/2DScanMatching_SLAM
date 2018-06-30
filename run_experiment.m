addpath(genpath(pwd))

#pkg install -forge <pkg-name>
pkg load statistics # chi2inv, etc.
pkg load octclip # oc_polybool
pkg load geometry # intersectPolylines, etc.
pkg load signal # ahsm

#slam
test_scan_matcher


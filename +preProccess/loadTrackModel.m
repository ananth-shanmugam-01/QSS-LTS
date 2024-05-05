function [trackDist, curvatureSpline] = loadTrackModel(Track,sector_dist)

    trackDist = 0:sector_dist:Track.sLap(end);
    curvatureSpline = csaps(Track.sLap, Track.Curv,0.8,trackDist);


end

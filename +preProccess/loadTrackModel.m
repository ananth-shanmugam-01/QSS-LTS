function [trackDist, curvatureSpline] = loadTrackModel(fileName,sector_dist)

    load(fileName);

    trackDist = 0:sector_dist:TrackData.trackDistance(end);
    curvatureSpline = csaps(TrackData.trackDistance, TrackData.trackCurvature,0.8,trackDist);


end

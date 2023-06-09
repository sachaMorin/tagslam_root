for i in 0 4 8 12
do
	rosrun tagslam make_tag.py --nx 2 --ny 2 --tsize 0.3 --tspace 0.25 --borderbits 1 --marginx 0.036598 --marginy 0.0295473 --draw_box --no-symm_corners --startid $i
	mv target.pdf $i.pdf
done

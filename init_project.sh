#! /bin/bash
erb $(pwd)/models/train_model/model.rsdf > $(pwd)/models/train_model/model.sdf
cp -r $(pwd)/models/* ~/.gazebo/models/

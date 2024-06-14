
gnome-terminal --tab -- bash -c "source /home/tlips/miniconda3/bin/activate && conda activate gello-pt  && cd /home/tlips/Code/SensorCommDDS/sensor_comm_dds && python -m communication.readers.irtouch32_ble_reader"
sleep 1
gnome-terminal --tab -- bash -c "source /home/tlips/miniconda3/bin/activate && conda activate gello-pt  && cd /home/tlips/Code/SensorCommDDS/sensor_comm_dds && python -m communication.readers.accelnet_ble_reader"
gnome-terminal --tab -- bash -c "source /home/tlips/miniconda3/bin/activate && conda activate gello-pt  && cd /home/tlips/Code/SensorCommDDS/sensor_comm_dds && python -m visualisation.visualisers.irtouch32_visualiser IRTouch32"


gnome-terminal --tab -- bash -c "source /home/tlips/miniconda3/bin/activate && conda activate gello-pt  && cd /home/tlips/Code/SensorCommDDS/sensor_comm_dds && python -m communication.readers.switches_ble_reader"
gnome-terminal --tab -- bash -c "source /home/tlips/miniconda3/bin/activate && conda activate gello-pt  && cd /home/tlips/Code/SensorCommDDS/sensor_comm_dds && python -m visualisation.visualisers.switches_visualiser Switches"
sleep 1
gnome-terminal --tab -- bash -c "source /home/tlips/miniconda3/bin/activate && conda activate gello-pt  && cd /home/tlips/Code/SensorCommDDS/sensor_comm_dds && python -m communication.readers.micmanip_serial_reader"
gnome-terminal --tab -- bash -c "source /home/tlips/miniconda3/bin/activate && conda activate gello-pt  && cd /home/tlips/Code/SensorCommDDS/sensor_comm_dds && python -m visualisation.visualisers.micmanip_spectrogram_visualiser MicManip"

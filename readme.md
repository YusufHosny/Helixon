# Helixon2 Research Project

In this repository, the software, dataset, and paper for the Helixon2 project can be found.

The paper in `./report_final` includes all the technical details and research decisions.

The algo-impl folder contains all the algorithms, evaluations, and models.
The data-preproc folder contains the data processing scripts, the unaligned dataset and the aligned dataset (in their respective folders).
The py-ardu-wifi folder contains the python and arduino software for collecting data and running the system in real time.
The report_final folder contains the final paper and the tex source.

# Quick Start
In order to run the evaluations, first install all the prerequisite external libraries (numpy, torch, etc.) and the helixon_hdf5io package using pip install -e or conda develop.
To use the real-time system additionally install the whole algo-impl folder as a package.
The data collection can be set up by programming an arduino as master and one as slave using the `#define` in the `interboard_comms.h`. Then setup the `.env` using the master arduino's ip and then running the python `__main__.py`. There are multiple datastreams available, for collecting data using TCP, UDP, and for running the system in real-time.
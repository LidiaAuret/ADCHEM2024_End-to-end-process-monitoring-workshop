# ADCHEM 2024 End-to-end process monitoring workshop: Repository

## Repository information

### Context
This repository is a fork of the End-to-end process monitoring [repository](https://github.com/Stellenbosch-University-Process-Eng/End-to-end-process-monitoring), specifically created for the [IFAC ADCHEM 2024](https://www.adchem2024.org/) workshop on the same topic. This repository also includes a Python implementation of the framework.

### Requirements
The repository contains MATLAB and Python implementations. There are some differences between the MATLAB and Python implementations, but the principles remain the same. Two numerical integration approaches (Euler and Runge-Kutta) are implemented for the Python approach.
- The MATLAB code was tested on R2023a and R2024a. The Statistical and Machine Learning Toolbox is required.
- The Python requirements can be found in requirements.txt in the root folder, suitable for conda/mamba environment creation.

### Contributions and bugs
Please feel free to contribute to the repository, and report any bugs or issues through the repository.

## Workshop information

### Workshop title
End-to-end process monitoring techniques and case studies: Hands-on workshop.

### Short summary
Practical process monitoring in industry requires fault detection, identification, diagnosis, and the implementation of process recovery actions. Algorithmic design and automation of all these steps are required if the future promise of more autonomous plants is to be realised. However, theoretical research in process monitoring is overwhelmingly focused on the task of fault detection. End-to-end process monitoring (Auret and Louw, 2023a) encompasses the fully automated workflow from detection to implementation of the appropriate corrective action in a process. This workshop will discuss and demonstrate a framework for improved experimentation of end-to-end process monitoring approaches. The workshop is hands-on, making use of open access case studies and example code with implementations available in both MATLAB and Python (Auret and Louw, 2023b). The workshop will cover the motivation for an end-to-end process monitoring simulation framework and give participants the opportunity to practice implementing the various components that make up the framework, including modular designs for process, control, sensors, actuators, process monitoring, operator actions, and economic performance assessment. The importance of variability and uncertainty for robust process monitoring design will be discussed and demonstrated in the case studies.

The expected outcomes of the workshop are:
- To understand the industrial importance of end-to-end process monitoring.
- To understand and be able to implement the components of an end-to-end process monitoring test bed (process, control, sensors, actuators, operator actions, process monitoring).
- To understand and be able to implement uncertainty and variability characteristics in an end-to-end process monitoring test bed.

### Tentative schedule
- 08h30 - 09h00: Welcome and introductions
- 09h00 - 10h00: (Presentation) Overview: Motivation and framework structure
- 10h00 - 10h30: (Interactive session) Computer setup and repository familiarization
- 10h30 - 11h00: Coffee break
- 11h00 - 12h00: (Presentation) Framework module details
- 12h00 - 12h30: (Interactive example) Framework familiarization and experiments
- 12h30 - 13h30: Lunch
- 13h30 - 14h30: (Presentation) Framework module details (continued)
- 14h30 - 15h30: (Interactive example) Framework familiarization and experiments
- 15h30 - 16h00: Coffee break
- 16h00 - 17h30: (Interactive challenge) Custom configurations and modules, improve process monitoring performance

### Intended audience
Process monitoring techniques demonstrated will typically be data-based approaches.

The workshop is aimed at:
- Graduate students new to the fields of process monitoring, fault diagnosis, fault tolerant control.
- Experienced researchers in aforementioned fields (and the field of reinforcement learning) looking to expand the complexity and industrial relevance of case studies used to demonstrate their techniques. 
- Industry practitioners interested in and working on problems related to industry 4.0, autonomous plants and decision-support systems may also benefit from the workshop and be able to contribute valuable insights to academic participants.

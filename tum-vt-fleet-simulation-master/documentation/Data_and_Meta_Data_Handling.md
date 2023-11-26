# When To Archive
* Paper is accepted in final form

# Data Archiving
* study_name should start with FS_ (for fleet simulation)
* TUM server location (TSL): ...
* TUM Workbench?
* TSL/confidential
 * TSL/confidential/study_name
* TSL/public
 * TSL/public/study_name

# Meta Data File
* saved in TUM server location TSL/metadata.xlsx (which format)
* data fields:
 * study_name
 * authors
 * title
 * doi (can be added later on as well)
 * research group
 * project (if part of project)
 * reference to gitlab commit (id, name, date)
 * abstract

# Simulation Result Data
* log files can become really large in debug mode -> debug mode should be off in "production"
* 1_user-stats.csv and 2-x_op-stats.csv can become really large; they are too large to be saved for thousands of simulations
* other files are typically not large and can be packed and saved in study_name/results.zip

# Study Evaluation Data
* both study evaluation scripts and final result data should be packed and saved study_name/evaluation.zip

# Input Data
* data should be available on server
 * split into publicly accessible and confidential data
 * problematic: overwriting of input data content -> data on server should remain the same (versioning?)
* scenario-csv files (i.e. all metadata required to replicate study) should be packed and copied to respective study_name/scenarios.zip

# Paper
* all manuscript data (Word File/TeX File with figures etc., Cover/Response Letter) of final version should be saved in 
 * study_name/paper.zip
* last state of paper in pdf should be saved in study_name as well


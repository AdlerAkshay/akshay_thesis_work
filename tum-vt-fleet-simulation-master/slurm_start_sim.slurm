#!/bin/bash
#SBATCH -J JOBNAME
#SBATCH -o LOG_DIRECTORY/%x.%j.%N.out
#SBATCH -D ./
#SBATCH --get-user-env
#SBATCH --clusters=cm2
#SBATCH --partition=cm2_std
#SBATCH --nodes=4
#SBATCH --tasks-per-node=2
#SBATCH --cpus-per-task=4
#SBATCH --mail-type=end
#SBATCH --mail-user=YOUR.NAME@tum.de
#SBATCH --export=NONE
#SBATCH --time=36:00:00
#SBATCH --array=0-1

module load slurm_setup
module load gurobi
module load python
source activate my_python

srun python slurm_run_job_array.py studies/STUDY_DIR/scenarios/CONSTANT_NAME.csv studies/STUDY_DIR/scenarios/SCENARIO_DEF_NAME.csv

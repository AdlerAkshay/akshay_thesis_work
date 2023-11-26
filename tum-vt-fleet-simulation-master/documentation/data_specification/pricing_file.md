# pricing_file.csv

* this file specifies the functionality used for dynamic pricing
* the name of the file can be adopted
* depending on the application there can be two versions of the pricing file
* either a time dependent pricing_file (given with the global "op_elastic_price_file")
* or a utilization dependent pricing_file (given with the global "op_util_surge_price_file")
* attribute fields of time dependent pricing file:

column_name | data_type | comment
-- | -- | --
time | int | start time of this pricing regime
base_fare_factor | float | factor of the base_fare in this pricing regime
distance_fare_factor | float | factor of the distance_fare in this pricing regime
general_factor | float | global price factor in this pricing regime

* attribute fields of utilization dependent pricing file:

column_name | data_type | comment
-- | -- | --
utilization | float | start utilization [0, 1] for this pricing regime
base_fare_factor | float | factor of the base_fare in this pricing regime
distance_fare_factor | float | factor of the distance_fare in this pricing regime
general_factor | float | global price factor in this pricing regime

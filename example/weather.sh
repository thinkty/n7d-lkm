#!/bin/sh

# A simple script querying the OpenWeatherMap for current temperature.
# Requires curl, jq
#
# @see https://openweathermap.org/current

# OpenWeatherMap API Key
AppId=""

# West Lafayette coordinates
Lat=40.4259
Lon=-86.9081

# Show temperature in celcius
Units="metric"

# Make the query and parse the feels_like temperature
Temperature=$(curl "https://api.openweathermap.org/data/2.5/weather?lat=$Lat&lon=$Lon&units=$Units&appid=$AppId" --silent | jq .main.feels_like)

# Check if the query failed
if [ -z $Temperature ]
then
	echo "Failed to fetch temperature data..."
else
	echo "Fetched temperature data: $Temperature"
	echo -n $Temperature > /dev/n7d
fi

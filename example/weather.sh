#!/bin/sh

# A simple script querying the OpenWeatherMap for current temperature.
# Requires curl, jq
#
# @see https://openweathermap.org/current

# OpenWeatherMap API Key
appId=""

# West Lafayette coordinates
lat=40.4259
lon=-86.9081

# Show temperature in celcius
units="metric"

# Make the query and parse the feels_like temperature
temperature=$(curl "https://api.openweathermap.org/data/2.5/weather?lat=$lat&lon=$lon&units=$units&appid=$appId" --silent | jq .main.feels_like)

# Check if the query failed
if [ $temperature = "null" ]
then
	echo "Failed to fetch temperature data..."
else
	echo "Fetched temperature data: $temperature"
	rounded=$(printf '%.0f' $temperature)
	echo "Rounded temperature: $rounded"
        echo -n "C$rounded" > /dev/n7d
fi

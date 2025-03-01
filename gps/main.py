import gps

def get_location():
    # Create a gps session
    session = gps.gps(mode=gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)

    try:
        while True:
            # Get the next GPS data from gpsd
            report = session.next()

            # Only process GPS data if it's a 'TPV' (time, position, velocity) report
            if report['class'] == 'TPV':
                lat = getattr(report, 'lat', None)
                lon = getattr(report, 'lon', None)

                if lat and lon:
                    print(f"Latitude: {lat}, Longitude: {lon}")
                else:
                    print("No GPS data available.")
                
    except KeyError:
        print("KeyError: Missing GPS data.")
    except KeyboardInterrupt:
        print("Program interrupted.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    get_location()

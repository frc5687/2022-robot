# Logbook

Use this as a file you can edit to make a new commit.  In your commit message, say "Tested - no bugs" or note what happened, e.g. "Tested - robot exploded but no human casualties".

## Entries

3/3/22 - Testing
3/3/22 - Climber progress:  The rocker arm Falcon is working (although we haven't added the MotionMagic code so we weren't able to control it).  The stationary-arm Falcon was not sending encoder information.  We swapped CAN ids and confirmed that the problem is the Falcon, not our logging clode.  Our best guess is that it ws configured to use an alternate encoder, but we ran out of time while researching that.  Further reading here (https://readthedocs.org/projects/phoenix-documentation/downloads/pdf/latest/) suggested that the solution is to configure to use the integral sensor, which I've done with this commit.  That will need to be tested in the morning.  If that works, we'll need to continue with the MotionMagic code.
3/7 testing fc7ec37e342b37659645101caf1bc5d418b02c69 -- may have broken robot. Forgot to uncomment hard coding of TwoBall auto
3/7 testing 0024f9b0065d863220143b127959d1683864db0d
 - position 1 zero ball working
 - position 2 zero ball working but in one run didn't get all the way out of tarmac
 - position 3 zero ball working
 - position 4 zero ball working
 - position unkonwn working (waits)
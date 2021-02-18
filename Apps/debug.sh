# Frame ctor - check NLeft value
b Frame.cc:401

# Tracking::GrabImageMonocular - after Frame creation
b Tracking.cc:228

# Tracking::Track() - start of fcn
b Tracking.cc:448

info breakpoints
run
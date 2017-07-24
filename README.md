# Parking-algorithm
Algorithm for parking in short time and high accuracy

The parking experiment gives the coordinates of the five parking spaces, requiring the algorithm to be programmed so that the vehicle can park into the parking space in the form of a vehicle at the rear of the vehicle or at the front. The parking spaces are 5 known parking spaces and one unknown parking space. Accuracy (the four vertices of the vehicle from the standard position of the deviation) as a criterion, the faster, the higher the accuracy, the more perfect algorithm.


Our direction control algorithm is in the form of PID, which is a point on the straight line that is perpendicular to the parking space and passes through the tail of the parking space. The coordinates are the opposite numbers of the odds of the distance between the vehicle and the parking space. The principle is that the vehicle The high order inverse scale curve is smoothly deposited, and the coordinates of the preview point are the intercepts of the tangent tangent on the curve. But adjust a lot of parameters, the effect is still bad. I think the main reason may be that we did not consider the length of the vehicle, simply to the vehicle as a particle, this explanation in line with the phenomenon of overkill. If we should change the control point from the vehicle center point to the rear point, control the rear storage, the effect should be better.


In the parking experiment we have tried a variety of programs, including 270 ° drift storage, the benefits of this approach is a short time, less process, but the drawback is obvious, that is, control error is very difficult, and finally because of time constraints and no Thinking and gave up the program, but I think this is a very interesting attempt.


In short, I think the main reason for this failure is the direction of the storage control is not done, and this is precisely the most critical point of parking experiments.

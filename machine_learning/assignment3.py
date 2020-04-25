import numpy as np 
import matplotlib.pyplot as plt
from sklearn.metrics.pairwise import rbf_kernel

cost_agd = []
def AdapativeGradientDescent(x1, x2, target):
	
	w = np.random.randn(4,1)
	w_o = w
	t_o = 0
	
	epsilon = 0.1
	target = target.T

	phi_vector = np.column_stack((np.ones((100,1), dtype=int), x1, x2, np.sqrt(x1**2+x2**2))).T 
	phi_sum = np.sum(phi_vector, axis=1)
	phi_sum = phi_sum.reshape((4,1))
	
	SumE_e = 0
	L      = 0.0001 #learning rate
	
	for iter in range(100):
		t = (1 + np.sqrt(1 + 4*t_o**2))/2
		v = w + ((t_o-1)/t)*(w-w_o)
		t_o = t

		SumE_e = 0
		y_vector=v.T @ phi_vector	
		e = y_vector - target
		
		#update w
		d_w = np.zeros((4,1)) 
		for n in range(100):
			if e[0][n] > epsilon:
				d_w += phi_vector[:,n].reshape(4,1)
			elif e[0][n] < - epsilon:
				d_w += -phi_vector[:,n].reshape(4,1)
		
		d_w = d_w + v
		w = v - L * d_w #updated W
		w_o = v
		#cost functin
		e_cost = abs(e) - epsilon
		for n in range(100):
			SumE_e += max (0, e_cost[0][n])
		cost_agd.append((SumE_e + w.T @ w / 2)[0])

	return y_vector, w


cost_gd = []
def GradientDescent(x1, x2, target):
	w = np.random.randn(4,1)
	epsilon = 0.1
	target = target.T

	phi_vector = np.column_stack((np.ones((100,1), dtype=int), x1, x2, np.sqrt(x1**2+x2**2))).T
	phi_sum = np.sum(phi_vector, axis=1)
	phi_sum = phi_sum.reshape((4,1))
	
	SumE_e = 0
	L      = 0.0001 #learning rate
	
	for iter in range(100):
		SumE_e = 0
		y_vector=w.T @ phi_vector	
		e = y_vector - target
		
		#update w
		d_w = np.zeros((4,1)) 
		for n in range(100):
			if e[0][n] > epsilon:
				d_w += phi_vector[:,n].reshape(4,1)
			elif e[0][n] < - epsilon:
				d_w += -phi_vector[:,n].reshape(4,1)
		
		d_w = d_w + w
		w = w - L * d_w #updated W

		#cost functin
		e_cost = abs(e) - epsilon
		for n in range(100):
			SumE_e += max (0, e_cost[0][n])
		cost_gd.append((SumE_e + w.T @ w / 2)[0])

	return y_vector, w

def test(x1, x2, w):
	phi_vector = np.column_stack((np.ones((100,1), dtype=int), x1, x2, np.sqrt(x1**2+x2**2))).T
	y_test=w.T@phi_vector
	return y_test

def Plot_Y_T(y, t, xlabel='t(n)', ylabel='y(n)', title='Expected vs Actual'):
	plt.scatter(t, y.T, c='green')
	plt.xlabel(xlabel)
	plt.ylabel(ylabel)
	plt.title(title)
	plt.show()


def PlotSamples(x1, x2, target, xlabel='x1', ylabel='x2', title='Generated Samples'):
	plt.scatter(x1, x2, c=target, s=100)
	plt.xlabel(xlabel)
	plt.ylabel(ylabel)
	plt.title(title)
	plt.show()


def GenerateSamples(number_samples=100):
	v1 = 0.7 * np.random.randn(number_samples,1) 
	v2 = 0.7 * np.random.randn(number_samples,1) 
	psi = np.random.uniform(np.pi, 5*np.pi, size=(number_samples,1))

	x1 = (psi + v1) * np.cos(psi + v2)
	x2 = (psi + v1) * np.sin(psi + v2)

	target = psi

	

	return x1, x2, target



def Task1():
	x1, x2, target = GenerateSamples()
	#plot color encoded target vector and smaples  
	PlotSamples(x1, x2, target, title='Generated smaples')

	#Gradient Descent 
	y, w_learned=GradientDescent(x1, x2, target) #train
	#Plot_Y_T(y, target)

	x1_test, x2_test, target_test = GenerateSamples() #test data
	y_test = test(x1_test, x2_test, w_learned) #predict using test data and learned W
	Plot_Y_T(y_test, target_test, title='Prediction GD') #plot - GD
	
	cost = np.array(cost_gd)
	print(cost.shape)
	plt.plot(cost)
	plt.xscale('log')
	plt.show()




	#Adaptive Gradient Descent
	y_ada , w_ada = AdapativeGradientDescent(x1, x2, target) #train

	x1_test_ada, x2_test_ada, target_test_ada = GenerateSamples() #test data
	y_test_ada = test(x1_test_ada, x2_test_ada, w_ada) #predict using test data and learned W
	Plot_Y_T(y_test_ada, target_test_ada, title='Prediction Adaptive GD') #plot - ada grad

	cost = np.array(cost_agd)
	print(cost.shape)
	plt.plot(cost)
	plt.xscale('log')
	plt.show()

def FISTA():

	w = np.random.randn(2, 100)
	w_0 = w 
	t_0 = 0

	a_b = np.vsplit(w_0, 2)
	a = a_b[0]
	b = a_b[1]


	learning_rate = 0.01
	x1, x2, target = GenerateSamples() #random samples


	for iter in range(2000):
		y, phi_x = trainAndTest_algorithm2(x1, x2, a, b) 
		#FISTA algorithm

		t = (1 + np.sqrt(1 + 4*t_0**2))/2
		v = w + ((t_0-1)/t)*(w-w_0)

		#Gradient computation
		grad_a = -0.5 * phi_x@phi_x.T@(a-b).T - np.full((100, 1), 0.1) + target

		grad_b = -0.5 * phi_x@phi_x.T@(b-a).T - np.full((100, 1), 0.1) - target 
		
		grad_w = np.concatenate((grad_a, grad_b), axis = 1)


		projection = v - learning_rate*grad_w.T

		w = np.clip(projection, 0, 1)

		t_0 = t
		w_0 = w
		w_update = np.vsplit(w_0, 2)
		a = w_update[0]
		b = w_update[1]
		
	Plot_Y_T(y, target)#plot trained y and target

	#Test - Prediction 
	x1, x2, target = GenerateSamples() #random samples
	y, phi_x = trainAndTest_algorithm2(x1, x2, a, b)
	Plot_Y_T(y, target, title='prediction-expected vs Actual : FISTA')#plot trained y and target


def optimize(a, b, y, learning_rate, K_xi_xj, target, number_samples=100):
	#v = w - eta * Gradient_cost
	w = np.concatenate((a, b), axis=0)

	grad_a = - K_xi_xj@(a-b).T - np.full((number_samples,1), 0.1) + target
	grad_b = - K_xi_xj@(b-a).T - np.full((number_samples,1), 0.1) - target
	# print("(a-b).T.shape = ", (a-b).T.shape)
	# print("grad_a.shape = ", grad_a.shape)
	# print("grad_b.shape = ", grad_b.shape)
	assert len(grad_a.shape) < 3
	grad_w = np.concatenate((grad_a, grad_b), axis=-1)
	v =  w - learning_rate * grad_w.T

	w_new = np.clip(v, 0, 1)

	#print('w_new', w_new)

	#print('w', np.shape(w_new))
	
	a_b = np.vsplit(w_new, 2)
	# print("w_new.shape = ", w_new.shape)
	# print("a_b[0].shape = ", a_b[0].shape)

	return a_b[0], a_b[1]

def trainAndTest_algorithm2(x1, x2, a, b):
	
	phi_x = np.column_stack((np.ones((100,1), dtype=int), x1, x2, np.sqrt(x1**2+x2**2)))

	y = abs(a - b)@phi_x@phi_x.T

	return y, phi_x

def Task2():
	x1, x2, target = GenerateSamples()
	#plot color encoded target vector and smaples  
	PlotSamples(x1, x2, target, title = 'samples')

	#Train algorithm
	a = np.random.randn(1, 100)
	b = np.random.randn(1, 100)
	learning_rate = 0.1

	#before training - just to test y and target data distribution
	y, phi_x = trainAndTest_algorithm2(x1, x2, a, b) 
	Plot_Y_T(y, target)#plot trained y and target 


	for iter in range(2000):
		y, phi_x = trainAndTest_algorithm2(x1, x2, a, b) 
		a, b = optimize(a, b, y, learning_rate, phi_x@phi_x.T, target)

	Plot_Y_T(y, target, title='Trained output')#plot trained y and target

	#After training 

	#Test - Prediction to see if it worked
	x1, x2, target = GenerateSamples() #random samples
	y, phi_x = trainAndTest_algorithm2(x1, x2, a, b) # test data with learnt parameters 
	Plot_Y_T(y, target, title='prediction-expected vs Actual : PGD')#plot trained y and target

	FISTA() #includes training and prediction
	


def trainAndTest_algorithm_RBF(x1, x2, a, b, sigma, target, number_samples=100):
	#k = np.exp(-np.linalg.norm(x1 - x2)**2/2*sigma**2)
	phi_x = np.column_stack([x1, x2])

	k = np.exp(-np.sum((phi_x[None, :, :] - phi_x[:, None, :])**2, axis=-1)/(2*sigma**2))
	#rbf_kernel()
	#print('k', np.shape(k))
	y = (a - b)@k
	return y, k

def RBFKernel():
	number_samples=1000
	x1, x2, target = GenerateSamples(number_samples)
	#plot color encoded target vector and smaples  
	PlotSamples(x1, x2, target, 'x1', 'x2')

	#Train algorithm
	a = np.random.randn(1, number_samples)
	b = np.random.randn(1, number_samples)
	learning_rate = 0.1
	sigma = 0.1

	#before training - just to test y and target data distribution
	y, k = trainAndTest_algorithm_RBF(x1, x2, a, b, sigma, target, number_samples) 
	Plot_Y_T(y, target,title='before training')#plot trained y and target 


	for iter in range(100):
		y, k = trainAndTest_algorithm_RBF(x1, x2, a, b, sigma, target, number_samples) 
		a, b = optimize(a, b, y, learning_rate, k, target, number_samples)

	Plot_Y_T(y, target, title='After training')#plot trained y and target

	#After training 

	#Test - Prediction to see if it worked
	x1, x2, target = GenerateSamples(number_samples) #random samples
	y, k = trainAndTest_algorithm_RBF(x1, x2, a, b, sigma, target, number_samples) # test data with learnt parameters 
	Plot_Y_T(y, target)#plot trained y and target


	#y, phi_x = trainAndTest_algorithm_RBF(x1, x2, a, b, 0.1) #sigma=0.1


def Task3():
	RBFKernel()
	#chnage the values of sigmaK and test here

def main():
	Task1()
	Task2()
	Task3()


if __name__ == '__main__':
	main()
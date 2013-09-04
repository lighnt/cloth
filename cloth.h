#ifndef CLOTH_H
#define CLOTH_H
#include <vector>
#include <Eigen/Eigen>
template <typename T> class clothProcess;
#define width 100
#define length 300

template <typename T>
class clothProcess
{
public:
	typedef Eigen::Matrix<T, 3, 1> PT3;
	typedef Eigen::Matrix<T, 2, 2> M22;
	clothProcess();
	void getObject();
	void setStep(T value);
private:
	void cacUVMatrix();
	std::vector<PT3, Eigen::aligned_allocator<PT3> > _pSet;
	std::vector<PT3, Eigen::aligned_allocator<PT3> > _speedSet;
	std::vector<PT3, Eigen::aligned_allocator<PT3> > _forceSet;
	std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > _triangleSet;
	std::vector<T> _stiffTriSet;
	std::vector<M22, Eigen::aligned_allocator<M22> > _uvMatrixSet;
	std::vector<Eigen::Vector4i, Eigen::aligned_allocator<Eigen::Vector4i> > _rectangleSet;
	std::vector<std::pair<T, T> > _stiffUVSet;//firt is u and second is v
	T step;

};

template <typename T>
clothProcess<T>::clothProcess()
{
	_pSet.clear();
	_triangleSet.clear();
	_rectangleSet.clear();
	_speedSet.clear();
	_forceSet.clear();
	_stiffTriSet.clear();
	_stiffUVSet.clear();
	_uvMatrixSet.clear();
}
template <typename T>
void clothProcess<T>::getObject()
{
	for(int i = 0; i < length; i++)
	{
		for(int j = 0; j < width; j++)
		{
			PT3 pnow(std::sqrt(T(i)), j, i);
			_pSet.push_back(pnow);
			if(i < length - 1 && j < width - 1)
			{
				Eigen::Vector3i tnow1(i*width + j, i*width + j + 1, (i + 1)*width + j+ 1);
				_triangleSet.push_back(tnow1);
				_stiffTriSet.push_back(T(1));
				Eigen::Vector3i tnow2(i*width + j, (i + 1)*width + j, (i + 1)*width + j + 1);
				_triangleSet.push_back(tnow2);
				_stiffTriSet.push_back(T(1));
				Eigen::Vector4i rnow1(i*width + j, i*width + j + 1, (i + 1)*width + j + 1, (i + 1)*width + j);
				_rectangleSet.push_back(rnow1);
				std::pair<T, T> uv1(T(1.0), T(1.0));
				_stiffUVSet.push_back(uv1);
				if(i < length - 2)
				{
					Eigen::Vector4i rnow2((i + 1)*width + j, i*width + j, (i + 1)*width + j + 1, (i + 2)*width + j + 1);
					_rectangleSet.push_back(rnow2);
					std::pair<T, T> uv2(T(1.0), T(1.0));
					_stiffUVSet.push_back(uv2);
				}
				if(i < width - 2)
				{
					Eigen::Vector4i rnow3(i*width + j + 1, i*width + j, (i + 1)*width + j + 1, (i + 1)*width + j + 2);
					_rectangleSet.push_back(rnow3);
					std::pair<T, T> uv3(T(1.0), T(1.0));
					_stiffUVSet.push_back(uv3);
				}
			}
			PT3 snow(T(0), T(0), T(0));
			_speedSet.push_back(snow);
			PT3 fnow(T(0), T(0), T(0));
			_forceSet.push_back(fnow);
		}
	}
	// set the initial force here
	_forceSet[0](2) = T(25);
	_forceSet[length - 1](2) = T(25);
	cacUVMatrix();
}
template <typename T>
void clothProcess<T>::cacUVMatrix()
{
	int p1, p2, p3;
	for(int i = 0; i < _triangleSet.size(); i++)
	{
		p1 = _triangleSet[i](0);
		p2 = _triangleSet[i](1);
		p3 = _triangleSet[i](2);
		M22 uv;
		uv(0, 0) = std::sqrt((_pSet[p2](0) - _pSet[p2](0))*(_pSet[p2](0) - _pSet[p1](0)) + (_pSet[p2](1) - _pSet[p1](1))*
			(_pSet[p2](1) - _pSet[p1](1)) + (_pSet[p2](2) - _pSet[p1](2))*(_pSet[p2](2) - _pSet[p1](2)));
		uv(1, 0) = T(0);
		uv(0, 1) = ((_pSet[p2](0) - _pSet[p1](0))*(_pSet[p3](0) - _pSet[p1](0)) + (_pSet[p2](1) - _pSet[p1](1))*(_pSet[p3](1) - _pSet[p1](1)) +
			(_pSet[p2](2) - _pSet[p1](2))*(_pSet[p3](2) - _pSet[p1](2)))/uv(0, 0);
		uv(1, 1) = std::sqrt((_pSet[p3](0) - _pSet[p1](0))*(_pSet[p3](0) - _pSet[p1](0)) + (_pSet[p3](1) - _pSet[p1](1))*
			(_pSet[p3](1) - _pSet[p1](1)) + (_pSet[p3](2) - _pSet[p1](2))*(_pSet[p3](2) - _pSet[p1](2)) - uv(0, 1)*uv(0,1));
		M22 uvInv = uv.inverse();
		_uvMatrixSet.push_back(unInv);
	}
}





#endif
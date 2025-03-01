using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;

public class PBD_model: MonoBehaviour {

	float 		t= 0.0333f;
	[Header ("damping force")]
	public float		damping= 0.99f; // for linear decay
	int[] 		E;
	float[] 	L;
	Vector3[] 	V;//velocities

	[Tooltip("-9.8f should be correct")]
	public Vector3 gravity = new Vector3(0.0f, -9.8f, 0.0f);
	public float restitution 	= 0.5f;					// for collision
	public float friction = 0.2f;

	Vector3 	sphere_center; //the location of the sphere
	Vector3 	sphere_velocity = new Vector3(0.0f, 0.0f, 0.0f); //the velocity of the sphere


	// Use this for initialization
	void Start () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		GameObject sphere = GameObject.Find("Sphere");

		//Resize the mesh.
		int n=21;
		Vector3[] X  	= new Vector3[n*n]; 
		Vector2[] UV 	= new Vector2[n*n]; //texture coordinates
		sphere_center = sphere.transform.position;
		int[] T	= new int[(n-1)*(n-1)*6];//triangle list
		for(int j=0; j<n; j++)
		for(int i=0; i<n; i++)
		{
			X[j*n+i] =new Vector3(5-10.0f*i/(n-1), 0, 5-10.0f*j/(n-1));
			UV[j*n+i]=new Vector3(i/(n-1.0f), j/(n-1.0f));
		}
		int t=0;
		for(int j=0; j<n-1; j++)
		for(int i=0; i<n-1; i++)	
		{
			T[t*6+0]=j*n+i;
			T[t*6+1]=j*n+i+1;
			T[t*6+2]=(j+1)*n+i+1;
			T[t*6+3]=j*n+i;
			T[t*6+4]=(j+1)*n+i+1;
			T[t*6+5]=(j+1)*n+i;
			t++;
		}
		mesh.vertices	= X;
		mesh.triangles	= T;
		mesh.uv 		= UV; 
		mesh.RecalculateNormals ();

		//Construct the original edge list
		int[] _E = new int[T.Length*2];
		for (int i=0; i<T.Length; i+=3) 
		{
			_E[i*2+0]=T[i+0];
			_E[i*2+1]=T[i+1];
			_E[i*2+2]=T[i+1];
			_E[i*2+3]=T[i+2];
			_E[i*2+4]=T[i+2];
			_E[i*2+5]=T[i+0];
		}
		//Reorder the original edge list
		for (int i=0; i<_E.Length; i+=2)
			if(_E[i] > _E[i + 1]) 
				Swap(ref _E[i], ref _E[i+1]);
		//Sort the original edge list using quicksort
		Quick_Sort (ref _E, 0, _E.Length/2-1);

		int e_number = 0;
		for (int i=0; i<_E.Length; i+=2) //only count the unique edges
			if (i == 0 || _E [i + 0] != _E [i - 2] || _E [i + 1] != _E [i - 1]) 
				e_number++;

		E = new int[e_number * 2];  //unique edge list(01,23,45,...)   #edges = E.length/2
		for (int i=0, e=0; i<_E.Length; i+=2)
			if (i == 0 || _E [i + 0] != _E [i - 2] || _E [i + 1] != _E [i - 1]) 
			{
				E[e*2+0]=_E [i + 0];
				E[e*2+1]=_E [i + 1];
				e++;
			}

		L = new float[E.Length/2]; //edge length
		for (int e=0; e<E.Length/2; e++) 
		{
			int i = E[e*2+0];
			int j = E[e*2+1];
			L[e]=(X[i]-X[j]).magnitude;
		}

		V = new Vector3[X.Length];
		for (int i=0; i<X.Length; i++)
			V[i] = new Vector3 (0, 0, 0);
	}

	void Quick_Sort(ref int[] a, int l, int r)
	{
		int j;
		if(l<r)
		{
			j=Quick_Sort_Partition(ref a, l, r);
			Quick_Sort (ref a, l, j-1);
			Quick_Sort (ref a, j+1, r);
		}
	}

	int  Quick_Sort_Partition(ref int[] a, int l, int r)
	{
		int pivot_0, pivot_1, i, j;
		pivot_0 = a [l * 2 + 0];
		pivot_1 = a [l * 2 + 1];
		i = l;
		j = r + 1;
		while (true) 
		{
			do ++i; while( i<=r && (a[i*2]<pivot_0 || a[i*2]==pivot_0 && a[i*2+1]<=pivot_1));
			do --j; while(  a[j*2]>pivot_0 || a[j*2]==pivot_0 && a[j*2+1]> pivot_1);
			if(i>=j)	break;
			Swap(ref a[i*2], ref a[j*2]);
			Swap(ref a[i*2+1], ref a[j*2+1]);
		}
		Swap (ref a [l * 2 + 0], ref a [j * 2 + 0]);
		Swap (ref a [l * 2 + 1], ref a [j * 2 + 1]);
		return j;
	}

	
	void Swap(ref int a, ref int b)
	{
		int temp = a;
		a = b;
		b = temp;
	}

	void Strain_Limiting()
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] vertices = mesh.vertices;

		//Apply PBD here.

		Vector3[] sum_x = new Vector3[vertices.Length];
		float[] sum_n = new float[vertices.Length];

		for	(int i = 0; i < vertices.Length; i++)
		{
			sum_x[i] = new Vector3(0.0f, 0.0f, 0.0f);
			sum_n[i] = 0;
		}

		for (int e = 0; e < E.Length / 2; e++) 
		{
			int i = E[e * 2 + 0];
			int j = E[e * 2 + 1];
			Vector3 sum = vertices[i] + vertices[j];  
			Vector3 diff = (vertices[i] - vertices[j]).normalized;
			sum_x[i] += 0.5f * (sum + L[e] * diff);
			sum_x[j] += 0.5f * (sum - L[e] * diff);
			sum_n[i]++;
			sum_n[j]++;
		}


		for (int i = 0; i < vertices.Length; i++)
		{	
			if (i == 0 || i == 20)	continue;
			Vector3 X_new = (0.2f * vertices[i] + sum_x[i]) / (0.2f + sum_n[i]);
			
			V[i] += (X_new - vertices[i]) / t;
			vertices[i] = X_new; 
		}

		mesh.vertices = vertices;
	}
	


	// void Collision_Handling()
	// {
	// 	Mesh mesh = GetComponent<MeshFilter> ().mesh;
	// 	Vector3[] X = mesh.vertices;
		
	// 	//For every vertex, detect collision and apply impulse if needed.
	// 	GameObject sphere = GameObject.Find("Sphere");
    //     Vector3 center = sphere.transform.position;

    //     float radius = 2.7f;
    //     for (int i = 0; i < X.Length; i++)
    //     {
    //         if (i == 0 || i == 20) continue;

	// 		Vector3 diff = transform.TransformPoint(X[i]) - center;
    //         if (diff.magnitude < radius)
    //         {
	// 			Vector3 x_new = center + radius * diff.normalized;
	// 			V[i] = (x_new - X[i]) / t;
	// 			X[i] = x_new;				
    //         }

	// 	}
	// 	mesh.vertices = X;
	// }


	//Another way to implement collision handling, this time we take fricition into account. 
	//This only work for circle mesh. Also, we don't consider the rotation.

	// void Collision_Handling()
	// {
	// 	Mesh mesh = GetComponent<MeshFilter> ().mesh;
	// 	Vector3[] X = mesh.vertices;
		
	// 	//For every vertex, detect collision and apply impulse if needed.
	// 	GameObject sphere = GameObject.Find("Sphere");
    //     Vector3 center = sphere.transform.position;

    //     float radius = 2.7f;
    //     for (int i = 0; i < X.Length; i++)
    //     {
    //         if (i == 0 || i == 20) continue;

	// 		Vector3 diff = transform.TransformPoint(X[i]) - center;
    //         if (diff.magnitude < radius)
    //         {
	// 			Vector3 x_new = center + radius * diff.normalized;

	// 			//Apply friction
	// 			Vector3 velocity_relative = V[i] - sphere_velocity;
	// 			if (Vector3.Dot(velocity_relative, diff.normalized) < 0)
	// 			{					
	// 				Vector3 Vn = Vector3.Dot(velocity_relative, diff.normalized) * diff.normalized;
	// 				Vector3 Vt = velocity_relative - Vn;
	// 				float a = Math.Max(0.0f, 1.0f - friction * (1 + restitution) * Vn.magnitude / Vt.magnitude);
	// 				Vector3 Vn_new = -restitution * Vn;
	// 				Vector3 Vt_new = a * Vt;
	// 				velocity_relative = Vn_new + Vt_new;
	// 				V[i] = velocity_relative + sphere_velocity;
	// 			}
	// 			else
	// 			{
	// 				V[i] = (x_new - X[i]) / t;
	// 			}			
	// 		X[i] = x_new;	
	// 		}			
            

	// 	}
	// 	mesh.vertices = X;
	// }



	// A more general collision handling function can be implemented here.
	// Compute the gradient of circle mesh. If you want to implement a different shape, you can modify the SDF and Gradient functions.
	

	//Circle
	float SDF(Vector3 a, Vector3 center, float radius)
	{	
		a -= center;
		return a.magnitude - radius;
	}
	
	Vector3 Gradient(Vector3 a, Vector3 center, float radius)
	{	
		Vector3 diff = a - center;
		float dist = diff.magnitude;
		Vector3 gradient = new Vector3(diff.x / dist, diff.y / dist, diff.z / dist);
		return gradient;
	}
	

	// Ellipsoid
	// float SDF(Vector3 a, Vector3 center, float radius)
	// {	
	// 	a = a - center;
	// 	a = new Vector3(a.x, a.y / 2, a.z);
	// 	return a.magnitude - radius;
	// }
	
	// Vector3 Gradient(Vector3 a, Vector3 center, float radius)
	// {	
	// 	Vector3 diff = a - center;
	// 	float dist = diff.magnitude;
	// 	Vector3 gradient = new Vector3(diff.x / dist, 2.0f * diff.y / dist, diff.z / dist);
	// 	return gradient;
	// }



	//Square
	// float SDF(Vector3 a, Vector3 center, float edge_length)
	// {
	// 	Vector3 diff = a - center;
	// 	float x = Math.Abs(diff.x);
	// 	float y = Math.Abs(diff.y);
	// 	float z = Math.Abs(diff.z);
		
	// 	if (x > y && x > z)
	// 	{	
	// 		float inter_x = MathF.Sign(diff.x) * edge_length / 2.0f;
	// 		Vector3 intersection = new Vector3(inter_x, inter_x * diff.y / diff.x, inter_x * diff.z / diff.x); 
	// 		float sign = Math.Sign(Math.Abs(diff.x) - Math.Abs(intersection.x));
	// 		return sign * (diff - intersection).magnitude;
	// 	}
	// 	if (y > x && y > z)
	// 	{
	// 		float inter_y = MathF.Sign(diff.y) * edge_length / 2.0f;
	// 		Vector3 intersection = new Vector3(inter_y * diff.x / diff.y, inter_y, inter_y * diff.z / diff.y); 
	// 		float sign = Math.Sign(Math.Abs(diff.y) - Math.Abs(intersection.y));
	// 		return sign * (diff - intersection).magnitude;
	// 	}
	// 	else
	// 	{
	// 		float inter_z = MathF.Sign(diff.z) * edge_length / 2.0f;
	// 		Vector3 intersection = new Vector3(inter_z * diff.x / diff.z, inter_z * diff.y / diff.z, inter_z); 
	// 		float sign = Math.Sign(Math.Abs(diff.z) - Math.Abs(intersection.z));
	// 		return sign * (diff - intersection).magnitude;		
	// 	}
	// }

	// Vector3 Gradient(Vector3 a, Vector3 center, float edge_length)
	// {
	// 	Vector3 diff = a - center;
	// 	return diff.normalized;		
	// }

	void Collision_Handling()
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X = mesh.vertices;
		
		//For every vertex, detect collision and apply impulse if needed.
		GameObject sphere = GameObject.Find("Sphere");
        Vector3 center = sphere.transform.position;

        float radius = 2.7f;
        for (int i = 0; i < X.Length; i++)
        {
            if (i == 0 || i == 20) continue;
			Vector3 location = transform.TransformPoint(X[i]);
			float diff = SDF(location, center, radius);

            if (diff < 0.0f)
            {
				Vector3 x_new = location - diff * Gradient(location, center, radius);
				V[i] = (x_new - X[i]) / t;
				X[i] = x_new;				
            }

		}
		mesh.vertices = X;
	}






	// Update is called once per frame
	void Update () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X = mesh.vertices;

		//Update the velocity of the sphere
		GameObject sphere = GameObject.Find("Sphere");
        Vector3 new_center = sphere.transform.position;
		sphere_velocity = (new_center - sphere_center) / t;
		sphere_center = new_center;
		

		for(int i=0; i<X.Length; i++)
		{
			if(i==0 || i==20)	continue;
			//Initial Setup
			V[i] += t * gravity;
			V[i] *= damping;
			X[i] += t * V[i];

		}
		mesh.vertices = X;



		for (int l = 0; l < 32; l++)
			Strain_Limiting (); //PBD

		Collision_Handling ();
		
		mesh.RecalculateNormals ();

	}


}














































// using UnityEngine;
// using System.Collections;
// using System.Collections.Generic;
// using System;

// public class PBD_model: MonoBehaviour {

// 	float 		t= 0.0333f;
// 	[Header ("damping force")]
// 	public float		damping= 0.99f; // for linear decay
// 	int[] 		E;
// 	float[] 	L;
// 	Vector3[] 	V;//velocities

// 	[Tooltip("-9.8f should be correct")]
// 	public Vector3 gravity = new Vector3(0.0f, -9.8f, 0.0f);
// 	public float restitution 	= 0.5f;					// for collision
// 	public float friction = 0.2f;

// 	Vector3 	sphere_center; //the location of the sphere
// 	Vector3 	sphere_velocity = new Vector3(0.0f, 0.0f, 0.0f); //the velocity of the sphere


// 	// Use this for initialization
// 	void Start () 
// 	{
// 		Mesh mesh = GetComponent<MeshFilter> ().mesh;
// 		GameObject sphere = GameObject.Find("Sphere");

// 		//Resize the mesh.
// 		int n=21;
// 		Vector3[] X  	= new Vector3[n*n]; 
// 		Vector2[] UV 	= new Vector2[n*n]; //texture coordinates
// 		sphere_center = sphere.transform.position;
// 		int[] T	= new int[(n-1)*(n-1)*6];//triangle list
// 		for(int j=0; j<n; j++)
// 		for(int i=0; i<n; i++)
// 		{
// 			X[j*n+i] =new Vector3(5-10.0f*i/(n-1), 0, 5-10.0f*j/(n-1));
// 			UV[j*n+i]=new Vector3(i/(n-1.0f), j/(n-1.0f));
// 		}
// 		int t=0;
// 		for(int j=0; j<n-1; j++)
// 		for(int i=0; i<n-1; i++)	
// 		{
// 			T[t*6+0]=j*n+i;
// 			T[t*6+1]=j*n+i+1;
// 			T[t*6+2]=(j+1)*n+i+1;
// 			T[t*6+3]=j*n+i;
// 			T[t*6+4]=(j+1)*n+i+1;
// 			T[t*6+5]=(j+1)*n+i;
// 			t++;
// 		}
// 		mesh.vertices	= X;
// 		mesh.triangles	= T;
// 		mesh.uv 		= UV; 
// 		mesh.RecalculateNormals ();

// 		//Construct the original edge list
// 		int[] _E = new int[T.Length*2];
// 		for (int i=0; i<T.Length; i+=3) 
// 		{
// 			_E[i*2+0]=T[i+0];
// 			_E[i*2+1]=T[i+1];
// 			_E[i*2+2]=T[i+1];
// 			_E[i*2+3]=T[i+2];
// 			_E[i*2+4]=T[i+2];
// 			_E[i*2+5]=T[i+0];
// 		}
// 		//Reorder the original edge list
// 		for (int i=0; i<_E.Length; i+=2)
// 			if(_E[i] > _E[i + 1]) 
// 				Swap(ref _E[i], ref _E[i+1]);
// 		//Sort the original edge list using quicksort
// 		Quick_Sort (ref _E, 0, _E.Length/2-1);

// 		int e_number = 0;
// 		for (int i=0; i<_E.Length; i+=2) //only count the unique edges
// 			if (i == 0 || _E [i + 0] != _E [i - 2] || _E [i + 1] != _E [i - 1]) 
// 				e_number++;

// 		E = new int[e_number * 2];  //unique edge list(01,23,45,...)   #edges = E.length/2
// 		for (int i=0, e=0; i<_E.Length; i+=2)
// 			if (i == 0 || _E [i + 0] != _E [i - 2] || _E [i + 1] != _E [i - 1]) 
// 			{
// 				E[e*2+0]=_E [i + 0];
// 				E[e*2+1]=_E [i + 1];
// 				e++;
// 			}

// 		L = new float[E.Length/2]; //edge length
// 		for (int e=0; e<E.Length/2; e++) 
// 		{
// 			int i = E[e*2+0];
// 			int j = E[e*2+1];
// 			L[e]=(X[i]-X[j]).magnitude;
// 		}

// 		V = new Vector3[X.Length];
// 		for (int i=0; i<X.Length; i++)
// 			V[i] = new Vector3 (0, 0, 0);




//     }
	

// 	void Quick_Sort(ref int[] a, int l, int r)
// 	{
// 		int j;
// 		if(l<r)
// 		{
// 			j=Quick_Sort_Partition(ref a, l, r);
// 			Quick_Sort (ref a, l, j-1);
// 			Quick_Sort (ref a, j+1, r);
// 		}
// 	}

// 	int  Quick_Sort_Partition(ref int[] a, int l, int r)
// 	{
// 		int pivot_0, pivot_1, i, j;
// 		pivot_0 = a [l * 2 + 0];
// 		pivot_1 = a [l * 2 + 1];
// 		i = l;
// 		j = r + 1;
// 		while (true) 
// 		{
// 			do ++i; while( i<=r && (a[i*2]<pivot_0 || a[i*2]==pivot_0 && a[i*2+1]<=pivot_1));
// 			do --j; while(  a[j*2]>pivot_0 || a[j*2]==pivot_0 && a[j*2+1]> pivot_1);
// 			if(i>=j)	break;
// 			Swap(ref a[i*2], ref a[j*2]);
// 			Swap(ref a[i*2+1], ref a[j*2+1]);
// 		}
// 		Swap (ref a [l * 2 + 0], ref a [j * 2 + 0]);
// 		Swap (ref a [l * 2 + 1], ref a [j * 2 + 1]);
// 		return j;
// 	}

	
// 	void Swap(ref int a, ref int b)
// 	{
// 		int temp = a;
// 		a = b;
// 		b = temp;
// 	}

// 	void Strain_Limiting()
// 	{
// 		Mesh mesh = GetComponent<MeshFilter> ().mesh;
// 		Vector3[] vertices = mesh.vertices;

// 		//Apply PBD here.

// 		Vector3[] sum_x = new Vector3[vertices.Length];
// 		float[] sum_n = new float[vertices.Length];

// 		for	(int i = 0; i < vertices.Length; i++)
// 		{
// 			sum_x[i] = new Vector3(0.0f, 0.0f, 0.0f);
// 			sum_n[i] = 0;
// 		}

// 		for (int e = 0; e < E.Length / 2; e++) 
// 		{
// 			int i = E[e * 2 + 0];
// 			int j = E[e * 2 + 1];
// 			Vector3 sum = vertices[i] + vertices[j];  
// 			Vector3 diff = (vertices[i] - vertices[j]).normalized;
// 			sum_x[i] += 0.5f * (sum + L[e] * diff);
// 			sum_x[j] += 0.5f * (sum - L[e] * diff);
// 			sum_n[i]++;
// 			sum_n[j]++;
// 		}


// 		for (int i = 0; i < vertices.Length; i++)
// 		{	
// 			if (isEdgePoint(i))	continue;
// 			Vector3 X_new = (0.2f * vertices[i] + sum_x[i]) / (0.2f + sum_n[i]);
			
// 			V[i] += (X_new - vertices[i]) / t;
// 			vertices[i] = X_new; 
// 		}

// 		mesh.vertices = vertices;
// 	}
	


// 	// void Collision_Handling()
// 	// {
// 	// 	Mesh mesh = GetComponent<MeshFilter> ().mesh;
// 	// 	Vector3[] X = mesh.vertices;
		
// 	// 	//For every vertex, detect collision and apply impulse if needed.
// 	// 	GameObject sphere = GameObject.Find("Sphere");
//     //     Vector3 center = sphere.transform.position;

//     //     float radius = 2.7f;
//     //     for (int i = 0; i < X.Length; i++)
//     //     {
//     //         if (i == 0 || i == 20) continue;

// 	// 		Vector3 diff = transform.TransformPoint(X[i]) - center;
//     //         if (diff.magnitude < radius)
//     //         {
// 	// 			Vector3 x_new = center + radius * diff.normalized;
// 	// 			V[i] = (x_new - X[i]) / t;
// 	// 			X[i] = x_new;				
//     //         }

// 	// 	}
// 	// 	mesh.vertices = X;
// 	// }


// 	//Another way to implement collision handling, this time we take fricition into account. 
// 	//This only work for circle mesh. Also, we don't consider the rotation.

// 	// void Collision_Handling()
// 	// {
// 	// 	Mesh mesh = GetComponent<MeshFilter> ().mesh;
// 	// 	Vector3[] X = mesh.vertices;
		
// 	// 	//For every vertex, detect collision and apply impulse if needed.
// 	// 	GameObject sphere = GameObject.Find("Sphere");
//     //     Vector3 center = sphere.transform.position;

//     //     float radius = 2.7f;
//     //     for (int i = 0; i < X.Length; i++)
//     //     {
//     //         if (i == 0 || i == 20) continue;

// 	// 		Vector3 diff = transform.TransformPoint(X[i]) - center;
//     //         if (diff.magnitude < radius)
//     //         {
// 	// 			Vector3 x_new = center + radius * diff.normalized;

// 	// 			//Apply friction
// 	// 			Vector3 velocity_relative = V[i] - sphere_velocity;
// 	// 			if (Vector3.Dot(velocity_relative, diff.normalized) < 0)
// 	// 			{					
// 	// 				Vector3 Vn = Vector3.Dot(velocity_relative, diff.normalized) * diff.normalized;
// 	// 				Vector3 Vt = velocity_relative - Vn;
// 	// 				float a = Math.Max(0.0f, 1.0f - friction * (1 + restitution) * Vn.magnitude / Vt.magnitude);
// 	// 				Vector3 Vn_new = -restitution * Vn;
// 	// 				Vector3 Vt_new = a * Vt;
// 	// 				velocity_relative = Vn_new + Vt_new;
// 	// 				V[i] = velocity_relative + sphere_velocity;
// 	// 			}
// 	// 			else
// 	// 			{
// 	// 				V[i] = (x_new - X[i]) / t;
// 	// 			}			
// 	// 		X[i] = x_new;	
// 	// 		}			
            

// 	// 	}
// 	// 	mesh.vertices = X;
// 	// }



// 	// A more general collision handling function can be implemented here.
// 	// Compute the gradient of circle mesh. If you want to implement a different shape, you can modify the SDF and Gradient functions.
	

// 	//Circle
// 	float SDF(Vector3 a, Vector3 center, float radius)
// 	{	
// 		a -= center;
// 		return a.magnitude - radius;
// 	}
	
// 	Vector3 Gradient(Vector3 a, Vector3 center, float radius)
// 	{	
// 		Vector3 diff = a - center;
// 		float dist = diff.magnitude;
// 		Vector3 gradient = new Vector3(diff.x / dist, diff.y / dist, diff.z / dist);
// 		return gradient;
// 	}
	

// 	// Ellipsoid
// 	// float SDF(Vector3 a, Vector3 center, float radius)
// 	// {	
// 	// 	a = a - center;
// 	// 	a = new Vector3(a.x, a.y / 2, a.z);
// 	// 	return a.magnitude - radius;
// 	// }
	
// 	// Vector3 Gradient(Vector3 a, Vector3 center, float radius)
// 	// {	
// 	// 	Vector3 diff = a - center;
// 	// 	float dist = diff.magnitude;
// 	// 	Vector3 gradient = new Vector3(diff.x / dist, 2.0f * diff.y / dist, diff.z / dist);
// 	// 	return gradient;
// 	// }



// 	//Square
// 	// float SDF(Vector3 a, Vector3 center, float edge_length)
// 	// {
// 	// 	Vector3 diff = a - center;
// 	// 	float x = Math.Abs(diff.x);
// 	// 	float y = Math.Abs(diff.y);
// 	// 	float z = Math.Abs(diff.z);
		
// 	// 	if (x > y && x > z)
// 	// 	{	
// 	// 		float inter_x = MathF.Sign(diff.x) * edge_length / 2.0f;
// 	// 		Vector3 intersection = new Vector3(inter_x, inter_x * diff.y / diff.x, inter_x * diff.z / diff.x); 
// 	// 		float sign = Math.Sign(Math.Abs(diff.x) - Math.Abs(intersection.x));
// 	// 		return sign * (diff - intersection).magnitude;
// 	// 	}
// 	// 	if (y > x && y > z)
// 	// 	{
// 	// 		float inter_y = MathF.Sign(diff.y) * edge_length / 2.0f;
// 	// 		Vector3 intersection = new Vector3(inter_y * diff.x / diff.y, inter_y, inter_y * diff.z / diff.y); 
// 	// 		float sign = Math.Sign(Math.Abs(diff.y) - Math.Abs(intersection.y));
// 	// 		return sign * (diff - intersection).magnitude;
// 	// 	}
// 	// 	else
// 	// 	{
// 	// 		float inter_z = MathF.Sign(diff.z) * edge_length / 2.0f;
// 	// 		Vector3 intersection = new Vector3(inter_z * diff.x / diff.z, inter_z * diff.y / diff.z, inter_z); 
// 	// 		float sign = Math.Sign(Math.Abs(diff.z) - Math.Abs(intersection.z));
// 	// 		return sign * (diff - intersection).magnitude;		
// 	// 	}
// 	// }

// 	// Vector3 Gradient(Vector3 a, Vector3 center, float edge_length)
// 	// {
// 	// 	Vector3 diff = a - center;
// 	// 	return diff.normalized;		
// 	// }

// 	void Collision_Handling()
// 	{
// 		Mesh mesh = GetComponent<MeshFilter> ().mesh;
// 		Vector3[] X = mesh.vertices;
		
// 		//For every vertex, detect collision and apply impulse if needed.
// 		GameObject sphere = GameObject.Find("Sphere");
//         Vector3 center = sphere.transform.position;

//         float radius = 2.7f;
//         for (int i = 0; i < X.Length; i++)
//         {
//             if (isEdgePoint(i)) continue;
// 			Vector3 location = transform.TransformPoint(X[i]);
// 			float diff = SDF(location, center, radius);

//             if (diff < 0.0f)
//             {
// 				Vector3 x_new = location - diff * Gradient(location, center, radius);
// 				V[i] = (x_new - X[i]) / t;
// 				X[i] = x_new;				
//             }

// 		}
// 		mesh.vertices = X;
// 	}


// 	bool isEdgePoint(int i)
// 	{
//     // 判断是否在区间0-20
//     if (i >= 0 && i <= 20)
//         return true;
//     if (i >= 420 && i <= 440)
//         return true;
//     if (i % 21 == 0)
//         return true;
//     if ((i + 1) % 21 == 0)
//         return true;
//     return false;
// 	}


// 	// Update is called once per frame
// 	void Update () 
// 	{
// 		Mesh mesh = GetComponent<MeshFilter> ().mesh;
// 		Vector3[] X = mesh.vertices;

// 		//Update the velocity of the sphere
// 		GameObject sphere = GameObject.Find("Sphere");
//         Vector3 new_center = sphere.transform.position;
// 		sphere_velocity = (new_center - sphere_center) / t;
// 		sphere_center = new_center;
		

// 		for(int i=0; i<X.Length; i++)
// 		{
// 			if(isEdgePoint(i))	continue;
// 			//Initial Setup
// 			// V[i] += t * gravity;
// 			// V[i] *= damping;
// 			X[i] += t * V[i];

// 		}
// 		mesh.vertices = X;

// 		if(Input.GetKey("r"))
// 		{
// 			V[220] += new Vector3(0.0f, 10.0f, 0.0f);
// 		}

// 		for (int l = 0; l < 32; l++)
// 			Strain_Limiting (); //PBD

// 		Collision_Handling ();

// 		// List<float> numbers = new List<float> { X[220].y, X[221].y, X[222].y, X[223].y, X[224].y, X[225].y, X[226].y, X[227].y, X[228].y, X[229].y, X[230].y};
// 		// Console.WriteLine(string.Join(", ", numbers));
		
//     	print($"x0: {X[220].y}, x1: {X[221].y}, x2: {X[222].y}, x3: {X[223].y}, x4: {X[224].y}, x5: {X[225].y}, x6: {X[226].y}, x7: {X[227].y}, x8: {X[228].y}, x9: {X[229].y}, x10: {X[230].y}");	
       

// 		mesh.RecalculateNormals ();

// 	}


// }



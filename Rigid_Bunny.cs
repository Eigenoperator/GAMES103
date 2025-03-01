using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;

public class Rigid_Bunny : MonoBehaviour 
{
	bool launched 		= false;
	float dt 			= 0.015f;
	Vector3 v 			= new Vector3(0, 0, 0);	// velocity
	Vector3 w 			= new Vector3(0, 0, 0);	// angular velocity
	
	float mass;									// mass
	Matrix4x4 I_ref;							// reference inertia

	public float linear_decay	= 0.999f;				// for velocity decay
	public float angular_decay	= 0.98f;				
	public float restitution 	= 0.5f;					// for collision
	public float friction = 0.2f;

	public Vector3 gravity = new Vector3(0.0f, -9.8f, 0.0f);	// gravity


	// Use this for initialization(simple initialization of the inertia matrix)
	// void Start () 
	// {		
	// 	Mesh mesh = GetComponent<MeshFilter>().mesh;
	// 	Vector3[] vertices = mesh.vertices;

	// 	float m=1;
	// 	mass=0;
	// 	for (int i=0; i<vertices.Length; i++) 
	// 	{
	// 		mass += m;
	// 		float diag=m*vertices[i].sqrMagnitude;
	// 		I_ref[0, 0]+=diag;
	// 		I_ref[1, 1]+=diag;
	// 		I_ref[2, 2]+=diag;
	// 		I_ref[0, 0]-=m*vertices[i][0]*vertices[i][0];
	// 		I_ref[0, 1]-=m*vertices[i][0]*vertices[i][1];
	// 		I_ref[0, 2]-=m*vertices[i][0]*vertices[i][2];
	// 		I_ref[1, 0]-=m*vertices[i][1]*vertices[i][0];
	// 		I_ref[1, 1]-=m*vertices[i][1]*vertices[i][1];
	// 		I_ref[1, 2]-=m*vertices[i][1]*vertices[i][2];
	// 		I_ref[2, 0]-=m*vertices[i][2]*vertices[i][0];
	// 		I_ref[2, 1]-=m*vertices[i][2]*vertices[i][1];
	// 		I_ref[2, 2]-=m*vertices[i][2]*vertices[i][2];
	// 	}
	// 	I_ref [3, 3] = 1;
	// }

	//another (again) version of the initialization of the inertia matrix

	float signed_volume (Vector3 a, Vector3 b, Vector3 c)
	{
		return Vector3.Dot(Vector3.Cross(b - a, c - a), a) / 6.0f;
	}

	// float signed(Vector3 a, Vector3 b, Vector3 c)
	// {
	// 	Vector3 normal = Vector3.Cross(b - a, c - a);
	// 	float cross = Vector3.Dot(a, normal);
	// 	if (cross > 0) return 1;
	// 	else return -1;
	// }

	// float signed_volume(Vector3 a, Vector3 b, Vector3 c)
	// {
	// 	return signed(a, b, c) * Volume(a, b, c);
	// }


	void Start () 
	{		
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;
		int[] triangles = mesh.triangles;

		//compute the centroid
		Vector3 centroid = new Vector3(0, 0, 0);
		for (int i = 0; i < vertices.Length; i++)
			centroid += vertices[i];
		centroid /= vertices.Length;

		float rou = 1.0f;
		float volume_total = 0;


		I_ref = Matrix4x4.zero;
		for (int e = 0; e < triangles.Length / 3; e++)
		{	
			// mass += m;
			int i = triangles[e * 3 + 0];
			int j = triangles[e * 3 + 1];
			int k = triangles[e * 3 + 2];
			Vector3 a = vertices[i];
			Vector3 b = vertices[j];
			Vector3 c = vertices[k];
			// Vector3 a = vertices[i] - centroid;
			// Vector3 b = vertices[j] - centroid;
			// Vector3 c = vertices[k] - centroid;
			Vector3 d = a + b + c;
			float volume = signed_volume(a, b, c);
			volume_total += volume;
			I_ref[0, 0] += rou * volume /20 * (a[1] * a[1] + b[1] * b[1] + c[1] * c[1] + d[1] * d[1] + a[2] * a[2] + b[2] * b[2] + c[2] * c[2] + d[2] * d[2]);
			I_ref[1, 1] += rou * volume /20 * (a[2] * a[2] + b[2] * b[2] + c[2] * c[2] + d[2] * d[2] + a[0] * a[0] + b[0] * b[0] + c[0] * c[0] + d[0] * d[0]);
			I_ref[2, 2] += rou * volume /20 * (a[0] * a[0] + b[0] * b[0] + c[0] * c[0] + d[0] * d[0] + a[1] * a[1] + b[1] * b[1] + c[1] * c[1] + d[1] * d[1]);
			I_ref[0, 1] -= rou * volume /20 * (a[0] * a[1] + b[0] * b[1] + c[0] * c[1] + d[0] * d[1] );
			I_ref[0, 2] -= rou * volume /20 * (a[0] * a[2] + b[0] * b[2] + c[0] * c[2] + d[0] * d[2] );
			I_ref[1, 2] -= rou * volume /20 * (a[1] * a[2] + b[1] * b[2] + c[1] * c[2] + d[1] * d[2] );			
		}   

		mass = volume_total * rou;
			
		I_ref[0,0] -= mass * centroid[1] * centroid[1] + mass * centroid[2] * centroid[2];
		I_ref[1,1] -= mass * centroid[2] * centroid[2] + mass * centroid[0] * centroid[0];
		I_ref[2,2] -= mass * centroid[0] * centroid[0] + mass * centroid[1] * centroid[1];
		I_ref[0,1] += mass * centroid[0] * centroid[1];
		I_ref[0,2] += mass * centroid[0] * centroid[2];
		I_ref[1,2] += mass * centroid[1] * centroid[2];

		I_ref[1, 0] = I_ref[0, 1];
		I_ref[2, 0] = I_ref[0, 2];
		I_ref[2, 1] = I_ref[1, 2];
		I_ref [3, 3] = 1;


		Debug.Log(mass);
		Debug.Log(I_ref);
	}

	
	

	
	Quaternion Quaternion_Addtion(Quaternion q1, Quaternion q2)
	{
		Quaternion q = new Quaternion(q1.x + q2.x, q1.y + q2.y, q1.z + q2.z, q1.w + q2.w);
		return q;
	}

	Matrix4x4 Get_Cross_Matrix(Vector3 a)
	{
		//Get the cross product matrix of vector a
		Matrix4x4 A = Matrix4x4.zero;
		A [0, 0] = 0; 
		A [0, 1] = -a [2]; 
		A [0, 2] = a [1]; 
		A [1, 0] = a [2]; 
		A [1, 1] = 0; 
		A [1, 2] = -a [0]; 
		A [2, 0] = -a [1]; 
		A [2, 1] = a [0]; 
		A [2, 2] = 0; 
		A [3, 3] = 1;
		return A;
	}

	Matrix4x4 Matirx_Subtraction(Matrix4x4 A, Matrix4x4 B)
	{
		Matrix4x4 C = Matrix4x4.zero;
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
				C[i, j] = A[i, j] - B[i, j];
		return C;
	}

	Matrix4x4 Matrix_Scalar_Multiplication(Matrix4x4 A, float s)
	{
		Matrix4x4 B = Matrix4x4.zero;
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
				B[i, j] = A[i, j] * s;
		return B;
	}

	// In this function, update v and w by the impulse due to the collision with
	//a plane <P, N>
	void Collision_Impulse(Vector3 P, Vector3 N)
	{
		Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices;
		List<Vector3> collision_vertices = new List<Vector3>();
		Vector3 position = transform.position;
		Matrix4x4 rotation_matrix = Matrix4x4.Rotate(transform.rotation);

		// Find the vertices that are inside the collision plane, write them in the list collision_vertices
		for (int i = 0; i < vertices.Length; i++)
		{
			Vector3 vertex_i_position = transform.TransformPoint(vertices[i]); //position + rotation_matrix.MultiplyVector(vertices[i]);
			if (Vector3.Dot(vertex_i_position - P, N) < 0.0f)
			{
				collision_vertices.Add(vertices[i]);
			}
		}

		if (collision_vertices.Count == 0)
			return;

		// Compute the average collision vertex position and velocity
		Vector3 average_collision_vertex = new Vector3(0.0f, 0.0f, 0.0f);
		for (int i = 0; i <collision_vertices.Count; i++)		
			average_collision_vertex += collision_vertices[i];			
		average_collision_vertex /= collision_vertices.Count;
		Vector3 average_collision_vertex_position = rotation_matrix.MultiplyVector(average_collision_vertex);
		Vector3 average_collision_vertex_velocity = v + Vector3.Cross(w, average_collision_vertex_position);

		if (Vector3.Dot(average_collision_vertex_velocity,N) > 0.0f)
			return;
		// Compute the wanted velocity after the collision
		Vector3 average_collision_vertex_velocityN = Vector3.Dot(average_collision_vertex_velocity, N) * N;
		Vector3 average_collision_vertex_velocityT = average_collision_vertex_velocity - average_collision_vertex_velocityN;
		float a = Math.Max(0.0f, 1.0f - friction * (1.0f + restitution) * average_collision_vertex_velocityN.magnitude / average_collision_vertex_velocityT.magnitude);
		Vector3 average_collision_vertex_velocityN_new = -restitution * average_collision_vertex_velocityN;
		Vector3 average_collision_vertex_velocityT_new = a * average_collision_vertex_velocityT;
		Vector3 average_collision_vertex_velocity_new = average_collision_vertex_velocityN_new + average_collision_vertex_velocityT_new;

		// Compute the impulse
		Matrix4x4 Inertia = rotation_matrix * I_ref * Matrix4x4.Transpose(rotation_matrix);
		Matrix4x4 Inertia_inverse = Matrix4x4.Inverse(Inertia);
		Matrix4x4 average_collision_vertex_position_cross_matrix = Get_Cross_Matrix(average_collision_vertex_position);
		Matrix4x4 K = Matirx_Subtraction(Matrix_Scalar_Multiplication(Matrix4x4.identity, 1.0f / mass), average_collision_vertex_position_cross_matrix * Inertia_inverse * average_collision_vertex_position_cross_matrix);
		Vector3 impulse = K.inverse.MultiplyVector(average_collision_vertex_velocity_new - average_collision_vertex_velocity);

		// Update v and w
		v += 1.0f / mass * impulse;
		w += Inertia_inverse.MultiplyVector(Vector3.Cross(average_collision_vertex_position, impulse));
	}

	// Update is called once per frame
	void Update () 
	{
		//Game Control
		if(Input.GetKey("r"))
		{
			transform.position = new Vector3 (0, 0.2f, 0);
			restitution = 0.5f;
			launched=false;
		}
		if(Input.GetKey("l"))
		{
			v = new Vector3 (5, 2, 0);
			launched=true;
		}

		if (launched)
		{

			// Part I: Update velocities
			v += dt * gravity;
			v = v * linear_decay;
			w = w * angular_decay;
		


			// Part II: Collision Impulse
			Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
			Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

		
			// Part III: Update position & orientation
			Vector3 x0 = transform.position;
			Quaternion q0 = transform.rotation;
			Vector3 x = x0 + dt * v;
			Vector3 dw = 0.5f * dt * w;
			Quaternion qw = new Quaternion(dw.x, dw.y, dw.z, 0.0f);
			Quaternion q = Quaternion_Addtion(q0, qw * q0);

			// Part IV: Assign to the object
			transform.position = x;
			transform.rotation = q;
		}
	}
}
















//in memory of the old code

 //Another initialization of the inertia matrix
	// Vector3 Sort3(int a, int b, int c) //a < b < c
	// {
	// 	Vector3 v = new Vector3(a, b, c);
	// 	if (v.x > v.y)
	// 	{
	// 		float temp = v.x;
	// 		v.x = v.y;
	// 		v.y = temp;
	// 	}
	// 	if (v.y > v.z)
	// 	{
	// 		float temp = v.y;
	// 		v.y = v.z;
	// 		v.z = temp;
	// 	}
	// 	if (v.x > v.y)
	// 	{
	// 		float temp = v.x;
	// 		v.x = v.y;
	// 		v.y = temp;
	// 	}
	// 	return v;
	// }


	// float Determination(Vector3 a, Vector3 b, Vector3 c)
	// {
	// 	return a.x * b.y * c.z + a.y * b.z * c.x + a.z * b.x * c.y - a.z * b.y * c.x - a.y * b.x * c.z - a.x * b.z * c.y;
	// }


	// bool HaveTwoCommonElements(Vector3 v1, Vector3 v2)
    // {

    //     int[] v1Elements = new int[] { (int)v1.x, (int)v1.y, (int)v1.z };
    //     int[] v2Elements = new int[] { (int)v2.x, (int)v2.y, (int)v2.z };


    //     int commonCount = 0;
    //     foreach (var element in v1Elements)
    //     {
    //         if (System.Array.Exists(v2Elements, e => e == element))
    //         {
    //             commonCount++;
    //         }
    //     }

    //     return commonCount == 2;
    // }

	// List<int> GetCommonElements(Vector3 v1, Vector3 v2)
	// {
	// 	int[] v1Elements = new int[] { (int)v1.x, (int)v1.y, (int)v1.z };
	// 	int[] v2Elements = new int[] { (int)v2.x, (int)v2.y, (int)v2.z };
	// 	List<int> commonElements = new List<int>();
	// 	foreach (var element in v1Elements)
	// 	{
	// 		if (System.Array.Exists(v2Elements, e => e == element))
	// 		{
	// 			commonElements.Add(element);
	// 		}
	// 	}
	// 	return commonElements;
	// }
	
	// bool Rotation_Consistant(Vector3 v1, Vector3 v2, int r1, int r2)//v1 and v2 are neibor triangles, r1 and r2 can only be 0 or 1
	// {
	// 	List<int> commonElements = GetCommonElements(v1, v2);
	// 	int a = commonElements[0];
	// 	int b = commonElements[1];
	// 	int a1, a2, b1, b2;
	// 	if (v1.x == a) a1 = 0;else if (v1.y == a) a1 = 1;else a1 = 2;
	// 	if (v1.x == b) b1 = 0;else if (v1.y == b) b1 = 1;else b1 = 2;
	// 	if (v2.x == a) a2 = 0;else if (v2.y == a) a2 = 1;else a2 = 2;
	// 	if (v2.x == b) b2 = 0;else if (v2.y == b) b2 = 1;else b2 = 2;
	// 	if (r1 == r2)
	// 	{
	// 		if (a1 == 0 && a2 == 1 || a1 == 1 && a2 == 2 || a1 == 2 && a2 == 0)
	// 			if (b1 == 0 && b2 == 1 || b1 == 1 && b2 == 2 || b1 == 2 && b2 == 0)
	// 				return false;
	// 			else
	// 				return true;
	// 		else
	// 			if (b1 == 0 && b2 == 1 || b1 == 1 && b2 == 2 || b1 == 2 && b2 == 0)
	// 				return true;
	// 			else
	// 				return false;
	// 	}
	// 	else
	// 	{
	// 		if (a1 == 0 && a2 == 1 || a1 == 1 && a2 == 2 || a1 == 2 && a2 == 0)
	// 			if (b1 == 0 && b2 == 1 || b1 == 1 && b2 == 2 || b1 == 2 && b2 == 0)
	// 				return true;
	// 			else
	// 				return false;
	// 		else
	// 			if (b1 == 0 && b2 == 1 || b1 == 1 && b2 == 2 || b1 == 2 && b2 == 0)
	// 				return false;
	// 			else
	// 				return true;
	// 	}
	// }	
	// Vector3 triangle_normal(Vector3 a, Vector3 b, Vector3 c)
	// {
	// 	Vector3 normal = Vector3.Cross(b - a, c - a);
	// 	normal.Normalize();
	// 	return normal;
	// }
	// float triangle_sign(Vector3 a, Vector3 b, Vector3 c)//This will return 1 or -1
	// {
	// 	Vector3 normal = triangle_normal(a, b, c);
	// 	float cross = Vector3.Dot(a, normal);
	// 	if (cross > 0) return 1;
	// 	else return -1;		
	// }

	// float signed_volume(Vector3 a, Vector3 b, Vector3 c)
	// {
	// 	return triangle_sign(a, b, c) * Determination(a, b, c) / 6;
	// }

	// void Start ()
	// {
	// 	Mesh mesh = GetComponent<MeshFilter>().mesh;
	// 	Vector3[] vertices = mesh.vertices;
	// 	int[] triangles = mesh.triangles;
	// 	List<int> triangles_visited = new List<int>();
	// 	Vector3[] triangles_lists = new Vector3[triangles.Length / 3]; //This is the triangle list
	// 	int[] triangles_orientation = new int[triangles.Length / 3]; //This can only be 0 or 1. 0 means from smallest to largest, 1 means from largest to smallest

	// 	//initialize the orientation of the triangles
	// 	for (int i = 0 ; i <triangles_orientation.Length; i++)
	// 	{
	// 		triangles_orientation[i] = 0;
	// 	}

	// 	//initialize the triangle list
	// 	for (int i = 0; i < triangles.Length / 3; i++)
	// 	{
	// 		triangles_lists[i] = Sort3(triangles[3 * i + 0], triangles[3 * i + 1], triangles[3 * i + 2]);
	// 	}

	// 	//change the orientation of the triangles
	// 	for (int i = 0; i < triangles.Length / 3; i++)
	// 	{	
	// 		//dont consider the triangles that have been visited
	// 		if (triangles_visited.Contains(i)) continue;
	// 		triangles_visited.Add(i);

	// 		//find the neibor triangles of i
	// 		for(int j = i + 1; j < triangles.Length / 3; j++)
	// 		{	
	// 			List<int> neibor_triangles = new List<int>();
	// 			if (triangles_visited.Contains(j)) continue;
	// 			if (HaveTwoCommonElements(triangles_lists[i], triangles_lists[j]))
	// 			{
	// 				triangles_visited.Add(j);
	// 				neibor_triangles.Add(j);
	// 			}
	// 			if (neibor_triangles.Count == 0) continue;
	// 			for (int k = 0; k < neibor_triangles.Count; k++)//update the orientation of the neibor triangles
	// 			{
	// 				if (Rotation_Consistant(triangles_lists[i], triangles_lists[neibor_triangles[k]], triangles_orientation[i], triangles_orientation[neibor_triangles[k]]))
	// 				{
	// 					triangles_orientation[neibor_triangles[k]] = 1 - triangles_orientation[neibor_triangles[k]];
	// 				}
	// 			}
	// 		}
	// 	}

	// 	//change the order of the vertices of the triangles
	// 	for (int i = 0; i < triangles.Length / 3; i++)
	// 	{
	// 		if (triangles_orientation[i] == 1)
	// 		{
	// 			int temp = triangles[3 * i + 1];
	// 			triangles[3 * i + 1] = triangles[3 * i + 2];
	// 			triangles[3 * i + 2] = temp;
	// 		}
	// 	}

	// 	//initialize the inertia matrix
	// 	for (int e = 0; e < triangles.Length; e++)
	// 	{
	// 		int i = triangles[e * 3 + 0];
	// 		int j = triangles[e * 3 + 1];
	// 		int k = triangles[e * 3 + 2];
	// 		Vector3 a = vertices[i];
	// 		Vector3 b = vertices[j];
	// 		Vector3 c = vertices[k];
	// 		Vector3 d = a + b + c;
	// 		float volume = signed_volume(a, b, c);
	// 		I_ref[0, 0] += volume /20 * (a.y * a.y + b.y * b.y + c.y * c.y +  + d.y * d.y + a.z * a.z + b.z * b.z + c.z * c.z + d.z * d.z);
	// 		I_ref[1, 1] += volume /20 * (a.z * a.z + b.z * b.z + c.z * c.z +  + d.z * d.z + a.x * a.x + b.x * b.x + c.x * c.x + d.x * d.x);
	// 		I_ref[2, 2] += volume /20 * (a.x * a.x + b.x * b.x + c.x * c.x +  + d.x * d.x + a.y * a.y + b.y * b.y + c.y * c.y + d.y * d.y);
	// 		I_ref[0, 1] -= volume /20 * (a.x * a.y + b.x * b.y + c.x * c.y +  + d.x * d.y );
	// 		I_ref[0, 2] -= volume /20 * (a.x * a.z + b.x * b.z + c.x * c.z +  + d.x * d.z );
	// 		I_ref[1, 2] -= volume /20 * (a.y * a.z + b.y * b.z + c.y * c.z +  + d.y * d.z );
	// 		I_ref[1, 0] = I_ref[0, 1];
	// 		I_ref[2, 0] = I_ref[0, 2];
	// 		I_ref[2, 1] = I_ref[1, 2];
	// 	}
	// 	I_ref [3, 3] = 1;

	// }
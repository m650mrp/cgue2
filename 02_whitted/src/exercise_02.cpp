#include <cglib/rt/renderer.h>
#include <cglib/rt/intersection_tests.h>
#include <cglib/rt/raytracing_context.h>
#include <cglib/rt/intersection.h>
#include <cglib/rt/ray.h>
#include <cglib/rt/scene.h>
#include <cglib/rt/light.h>
#include <cglib/rt/material.h>
#include <cglib/rt/render_data.h>

/*
 * TODO: implement a ray-sphere intersection test here.
 * The sphere is defined by its center and radius.
 *
 * Return true, if (and only if) the ray intersects the sphere.
 * In this case, also fill the parameter t with the distance such that
 *    ray_origin + t * ray_direction
 * is the intersection point.
 */
bool intersect_sphere(
    glm::vec3 const& ray_origin,    // starting point of the ray
    glm::vec3 const& ray_direction, // direction of the ray
    glm::vec3 const& center,        // position of the sphere
    float radius,                   // radius of the sphere
    float* t)                       // output parameter which contains distance to the hit point
{
    cg_assert(t);
	cg_assert(std::fabs(glm::length(ray_direction) - 1.f) < EPSILON);
	
	const float a = glm::dot(ray_direction, ray_direction);
	const float b = glm::dot(2.f * ray_direction, ray_origin - center);
	const float c = glm::dot(ray_origin - center, ray_origin - center) - radius * radius;
		       
	const float discriminant = b * b - 4 * a * c;
		 
	if(discriminant < 0) return false;
			     
	const float t1 = (-b - glm::sqrt(discriminant)) / (2 * a);
	const float t2 = (-b + glm::sqrt(discriminant)) / (2 * a);
				     
	if(t1 >=0.f && t2 >=0.f) *t = glm::min(t1, t2);
	   else if(t1 >=0.f) *t = t1;
	   else if(t2 >=0.f) *t = t2;
	   else return false;
				     
	  return true;



}

/*
 * emission characteristic of a spotlight
 */
glm::vec3 SpotLight::getEmission(
		glm::vec3 const& omega // world space direction
		) const
{
	cg_assert(std::fabs(glm::length(omega) - 1.f) < EPSILON);
 
	return power * (2 * falloff) * glm::pow(glm::max(0.f, glm::dot(omega, direction)), falloff);// TODO: implement a spotlight emitter as specified on the exercise sheet
	//return glm::vec3(0.f);
}

glm::vec3 evaluate_phong(
	RenderData &data,			// class containing raytracing information
	MaterialSample const& mat,	// the material at position
	glm::vec3 const& P,			// world space position
	glm::vec3 const& N,			// normal at the position (already normalized)
	glm::vec3 const& V)			// view vector (already normalized)
{
	cg_assert(std::fabs(glm::length(N) - 1.f) < EPSILON);
	cg_assert(std::fabs(glm::length(V) - 1.f) < EPSILON);

	glm::vec3 contribution(0.f);

	// iterate over lights and sum up their contribution
	for (auto& light_uptr : data.context.scene->lights) 
	{
		// TODO: calculate the (normalized) direction to the light
		const Light *light = light_uptr.get();
		glm::vec3 lightPos(light->getPosition());
		       
		glm::vec3 L(glm::normalize(lightPos - P));

		float visibility = 1.f;
		if (data.context.params.shadows) {
			if(!visible(data, P, lightPos))
				visibility = 0.f;// TODO: check if light source is visible
		}
		
		const glm::vec3 illumination = light->getEmission(-L) * visibility * glm::max(0.f, glm::dot(N, L)) / glm::dot(P - lightPos, P - lightPos);
		glm::vec3 diffuse(0.f);
		if (data.context.params.diffuse) {
			diffuse = illumination * mat.k_d;// TODO: compute diffuse component of phong model
		}

		glm::vec3 specular(0.f);
		if (data.context.params.specular) {
			glm::vec3 RL(reflect(L, N));
			specular = illumination * mat.k_s * glm::pow(glm::max(0.f, glm::dot(V, RL)), mat.n);// TODO: compute specular component of phong model
		}

		glm::vec3 ambient(0.0f);

		if(data.context.params.ambient) {
		ambient = light->getPower() / glm::dot(P - lightPos, P - lightPos) * mat.k_a;
		}// TODO: modify this and implement the phong model as specified on the exercise sheet
		contribution += diffuse + specular + ambient;;
	}

	return contribution;
}

glm::vec3 evaluate_reflection(
	RenderData &data,			// class containing raytracing information
	int depth,					// the current recursion depth
	glm::vec3 const& P,			// world space position
	glm::vec3 const& N,			// normal at the position (already normalized)
	glm::vec3 const& V)			// view vector (already normalized)
{
	// TODO: calculate reflective contribution by constructing and shooting a reflection ray.
	const auto& params = data.context.params; 
	glm::vec3 RV(reflect(V,N));
	return trace_recursive(data, Ray(P + params.ray_epsilon * RV, RV), depth + 1);
}

glm::vec3 evaluate_transmission(
	RenderData &data,			// class containing raytracing information
	int depth,					// the current recursion depth
	glm::vec3 const& P,			// world space position
	glm::vec3 const& N,			// normal at the position (already normalized)
	glm::vec3 const& V,			// view vector (already normalized)
	float eta)					// the relative refraction index
{
	// TODO: calculate transmissive contribution by constructing and shooting a transmission ray.
	glm::vec3 RV;
    if(!refract(V, N, eta, &RV))
        return glm::vec3(0.f);
 
    return trace_recursive(data, Ray(P + data.context.params.ray_epsilon * RV, RV), depth + 1);
}

glm::vec3 handle_transmissive_material_single_ior(
	RenderData &data,			// class containing raytracing information
	int depth,					// the current recursion depth
	glm::vec3 const& P,			// world space position
	glm::vec3 const& N,			// normal at the position (already normalized)
	glm::vec3 const& V,			// view vector (already normalized)
	float eta)					// the relative refraction index
{
	if (data.context.params.fresnel) {
		const float f = fresnel(V, N, eta);// TODO: replace with proper fresnel handling.
		return f* evaluate_transmission(data, depth, P, N, V, eta)+ (1.f - f) * evaluate_transmission(data, depth, P, N, V, eta);
	}
	else {
		// just regular transmission
		return evaluate_transmission(data, depth, P, N, V, eta);
	}
}

glm::vec3 handle_transmissive_material(
	RenderData &data,					// class containing raytracing information
	int depth,							// the current recursion depth
	glm::vec3 const& P,					// world space position
	glm::vec3 const& N,					// normal at the position (already normalized)
	glm::vec3 const& V,					// view vector (already normalized)
	glm::vec3 const& eta_of_channel)	// relative refraction index of red, green and blue color channel
{
	if (data.context.params.dispersion && !(eta_of_channel[0] == eta_of_channel[1] && eta_of_channel[0] == eta_of_channel[2])) {
		// TODO: split ray into 3 rays (one for each color channel) and implement dispersion here
		return glm::vec3(
	                handle_transmissive_material_single_ior(data, depth, P, N, V, eta_of_channel[0]).x,
	                handle_transmissive_material_single_ior(data, depth, P, N, V, eta_of_channel[1]).y,
	                handle_transmissive_material_single_ior(data, depth, P, N, V, eta_of_channel[2]).z);	}
	else {
		// dont handle transmission, take average refraction index instead.
		const float eta = 1.f/3.f*(eta_of_channel[0]+eta_of_channel[1]+eta_of_channel[2]);
		return handle_transmissive_material_single_ior(data, depth, P, N, V, eta);
	}
	return glm::vec3(0.f);
}
// CG_REVISION 9a8fb58b6152e821391cc224d74a0db70c322327

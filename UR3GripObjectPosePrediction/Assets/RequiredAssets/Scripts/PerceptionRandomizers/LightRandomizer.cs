using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using UnityEngine.Perception.Randomization.Randomizers;
using UnityEngine.Perception.Randomization.Samplers;
using UnityEngine.Rendering.HighDefinition;

using PerceptionRandParameters = UnityEngine.Perception.Randomization.Parameters;

[Serializable]
[AddRandomizerMenu("Perception/Light Randomizer")]
public class LightRandomizer : Randomizer
{
    public PerceptionRandParameters.FloatParameter lightIntensityParameter = new PerceptionRandParameters.FloatParameter{ value = new UniformSampler(1000f, 80000f)}; // .9f, 1.1f

    public PerceptionRandParameters.FloatParameter rotationX = new PerceptionRandParameters.FloatParameter { value = new UniformSampler(40, 80)};

    public PerceptionRandParameters.FloatParameter rotationY = new PerceptionRandParameters.FloatParameter { value = new UniformSampler(-180, 180)};

    public PerceptionRandParameters.ColorRgbParameter lightColorParameter = new PerceptionRandParameters.ColorRgbParameter
    {
        red = new UniformSampler(.5f, 1f),
        green = new UniformSampler(.5f, 1f),
        blue = new UniformSampler(.5f, 1f),
        alpha = new ConstantSampler(1f)
    };

    // public PerceptionRandParameters.FloatParameter temperatureParameter = new PerceptionRandParameters.FloatParameter { value = new UniformSampler(1500f, 15000f) };

    protected override void OnIterationStart()
    {
        /*Runs at the start of every iteration*/
        IEnumerable<LightRandomizerTag> tags = tagManager.Query<LightRandomizerTag>();

        foreach (LightRandomizerTag tag in tags)
        {
            if (tag.TryGetComponent(out HDAdditionalLightData hdLight)) {
                // Randomize intensity
                float intensity = lightIntensityParameter.Sample();
                hdLight.SetIntensity(intensity, LightUnit.Lux);

                // Randomize color
                hdLight.color = lightColorParameter.Sample();
            }

            // Randomize rotation
            Vector3 rotation = new Vector3(rotationX.Sample(), rotationY.Sample(), tag.gameObject.transform.eulerAngles.z);
            tag.gameObject.transform.rotation = Quaternion.Euler(rotation);
        }

    }
}

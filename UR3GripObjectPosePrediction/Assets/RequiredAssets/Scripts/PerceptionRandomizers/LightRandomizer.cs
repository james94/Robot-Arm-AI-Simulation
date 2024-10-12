using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
// using UnityEngine.Perception.Randomization.Parameters;
using UnityEngine.Perception.Randomization.Randomizers;
using UnityEngine.Perception.Randomization.Samplers;
// using UnityEngine.Rendering;
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


                // // Set intensity based on light type
                // switch (hdLight.type) {
                //     case HDLightType.Directional:
                //         hdLight.SetIntensity(lightIntensityParameter.Sample(), LightUnit.Lux);
                //         break;
                //     case HDLightType.Point:
                //     case HDLightType.Spot:
                //         hdLight.SetIntensity(lightIntensityParameter.Sample(), LightUnit.Lumen);
                //         break;
                //     case HDLightType.Area:
                //         hdLight.SetIntensity(lightIntensityParameter.Sample(), LightUnit.Lumen);
                //         break;
                // }

                // Randomize color
                // if (hdLight.useColorTemperature) {
                //     // If using color temperature, randomize both color and temperature
                //         // Set color
                //     hdLight.color = lightColorParameter.Sample();
                //         // Set color temperature
                //     float temperature = temperatureParameter.Sample();
                //     hdLight.colorTemperature = temperature;

                //     // hdLight.SetColor(lightColorParameter.Sample());
                //     // hdLight.SetColorTemperature(temperatureParameter.Sample());
                // }
                // else {
                    // If not using color temperature, only randomize color
                hdLight.color = lightColorParameter.Sample();
                // hdLight.SetColor(lightColorParameter.Sample());
                // }

            //     hdLight.SetColor(lightColorParameter.Sample());

            //     Vector3 rotation = new Vector3(rotationX.Sample(), rotationY.Sample(), tag.transform.eulerAngles.z);
            //     tag.transform.rotation = Quaternion.Euler(rotation);
            }

            // var light = tag.gameObject.GetComponent<Light>();
            // light.intensity = lightIntensityParameter.Sample();
            // light.color = lightColorParameter.Sample();

            Vector3 rotation = new Vector3(rotationX.Sample(), rotationY.Sample(), tag.gameObject.transform.eulerAngles.z);
            tag.gameObject.transform.rotation = Quaternion.Euler(rotation);
        }

    }
}
